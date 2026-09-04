/* Copyright (C) 2022  Dmitry Serov
 *
 * This file is part of MControlCenter.
 *
 * MControlCenter is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MControlCenter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with MControlCenter. If not, see <https://www.gnu.org/licenses/>.
 */

#include "helper.h"
#include "msi-ec.h"
#include "readwrite.h"
#include <QCoreApplication>
#include <QDBusConnection>
#include <QDBusError>
#include <QFile>
#include <QProcess>
#include <QTimer>

ReadWrite rw;

const QString coolerBoostPath = "/sys/devices/platform/msi-ec/cooler_boost";
const QString ecFirmwareVersionPath = "/sys/devices/platform/msi-ec/fw_version";
const QString cpuEnergyPath = "/sys/class/powercap/intel-rapl-mmio:0/energy_uj";
const QString cpuMaxEnergyRangePath = "/sys/class/powercap/intel-rapl-mmio:0/max_energy_range_uj";
const QString cpuPowerLimitPath = "/sys/class/powercap/intel-rapl-mmio:0/constraint_0_power_limit_uw";
const QByteArray supportedEcFirmwareVersion = "14C6EMS1.109";
const qint64 maximumCpuEnergySampleIntervalMs = 5000;

void Helper::quit() const {
    QTimer::singleShot(0, QCoreApplication::instance(), &QCoreApplication::quit);
}

QByteArray Helper::getData() const {
    return rw.readFromFile();
}

void Helper::putValue(const int &address, const int &value) const {
    if (value >= 0 && value <= 255)
        rw.writeToFile(address, value);
    else
        fprintf(stderr, "tried to input invalid value. Address: %d, value: %d\n", address, value);
}

double Helper::getCpuPackagePower() {
    QFile cpuEnergyFile(cpuEnergyPath);
    if (!cpuEnergyFile.open(QIODevice::ReadOnly)) {
        cpuEnergySampleTimer.invalidate();
        return -1.0;
    }

    bool validCpuEnergy = false;
    const quint64 currentCpuEnergyUj = cpuEnergyFile.readAll().trimmed().toULongLong(&validCpuEnergy);
    if (!validCpuEnergy) {
        cpuEnergySampleTimer.invalidate();
        return -1.0;
    }

    if (!cpuEnergySampleTimer.isValid()) {
        previousCpuEnergyUj = currentCpuEnergyUj;
        cpuEnergySampleTimer.start();
        return -1.0;
    }

    const qint64 elapsedMilliseconds = cpuEnergySampleTimer.restart();
    const quint64 previousEnergyUj = previousCpuEnergyUj;
    previousCpuEnergyUj = currentCpuEnergyUj;
    if (elapsedMilliseconds <= 0 || elapsedMilliseconds > maximumCpuEnergySampleIntervalMs)
        return -1.0;

    quint64 consumedEnergyUj = 0;
    if (currentCpuEnergyUj >= previousEnergyUj) {
        consumedEnergyUj = currentCpuEnergyUj - previousEnergyUj;
    } else {
        QFile maxEnergyRangeFile(cpuMaxEnergyRangePath);
        if (!maxEnergyRangeFile.open(QIODevice::ReadOnly))
            return -1.0;

        bool validMaxEnergyRange = false;
        const quint64 maxEnergyRangeUj = maxEnergyRangeFile.readAll().trimmed().toULongLong(&validMaxEnergyRange);
        if (!validMaxEnergyRange || previousEnergyUj > maxEnergyRangeUj || currentCpuEnergyUj > maxEnergyRangeUj)
            return -1.0;

        consumedEnergyUj = maxEnergyRangeUj - previousEnergyUj + currentCpuEnergyUj;
    }

    return static_cast<double>(consumedEnergyUj)
           / (static_cast<double>(elapsedMilliseconds) * 1000.0);
}

bool Helper::isPowerLimitControlSupported() const {
    if (!QFile::exists(cpuPowerLimitPath))
        return false;

    QFile ecFirmwareVersionFile(ecFirmwareVersionPath);
    return ecFirmwareVersionFile.open(QIODevice::ReadOnly)
           && ecFirmwareVersionFile.readAll().trimmed() == supportedEcFirmwareVersion;
}

void Helper::enforcePowerLimit(const int &watts, const bool &onlyWhenCoolerBoost) const {
    // Keep the D-Bus API narrow: callers select a sane value in watts, while
    // the privileged helper owns the fixed sysfs paths and unit conversion.
    if (!isPowerLimitControlSupported() || watts < 1 || watts > 500)
        return;

    if (onlyWhenCoolerBoost) {
        QFile coolerBoostFile(coolerBoostPath);
        if (!coolerBoostFile.open(QIODevice::ReadOnly) || coolerBoostFile.readAll().trimmed() != "on")
            return;
    }

    const qint64 requestedPowerLimit = static_cast<qint64>(watts) * 1000000;
    QFile powerLimitFile(cpuPowerLimitPath);
    if (!powerLimitFile.open(QIODevice::ReadOnly))
        return;

    bool validCurrentPowerLimit = false;
    const qint64 currentPowerLimit = powerLimitFile.readAll().trimmed().toLongLong(&validCurrentPowerLimit);
    powerLimitFile.close();
    if (!validCurrentPowerLimit || currentPowerLimit == requestedPowerLimit)
        return;

    if (powerLimitFile.open(QIODevice::WriteOnly))
        powerLimitFile.write(QByteArray::number(requestedPowerLimit));
}

bool Helper::isEcSysModuleLoaded() const {
    if (rw.isEcSys()) {
        return true;
    }
    if (rw.isAcpiEc()) {
        fprintf(stderr, "%s\n", qPrintable("The acpi_ec kernel module is loaded"));
        return true;
    }
    fprintf(stderr, "%s\n", qPrintable("The ec_sys kernel module is not loaded"));
    return false;
}

bool Helper::loadEcSysModule() const {
    fprintf(stderr, "%s\n", qPrintable("Trying to load the ec_sys kernel module"));
    auto *process = new QProcess();
    process->start("sh", QStringList() << "-c" << "/usr/sbin/modprobe ec_sys write_support=1 2>&1");
    process->waitForFinished(1000);
    if (QByteArray output = process->readAllStandardOutput(); output != "")
        fprintf(stderr, "%s", qPrintable(output));
    if (isEcSysModuleLoaded())
        return true;
    return false;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    QObject obj;
    auto *helper = new Helper(&obj);
    QObject::connect(&a, &QCoreApplication::aboutToQuit, helper, &Helper::aboutToQuit);
    helper->setProperty("value", "initial value");
    QDBusConnection::systemBus().registerObject("/", &obj);

    QObject objMsiEc;
    auto *helperMsiEc = new MsiEc(&objMsiEc);
    QDBusConnection::systemBus().registerObject("/msi_ec", &objMsiEc);

    if (!QDBusConnection::systemBus().registerService(SERVICE_NAME)) {
        fprintf(stderr, "%s\n", qPrintable(QDBusConnection::systemBus().lastError().message()));
        exit(1);
    }

    return QCoreApplication::exec();
}
