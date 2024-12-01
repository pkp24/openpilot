#include <filesystem>

#include "selfdrive/frogpilot/ui/qt/offroad/utilities.h"

UtilitiesPanel::UtilitiesPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent) {
  ButtonControl *flashPandaBtn = new ButtonControl(tr("Flash Panda"), tr("FLASH"), tr("Flashes the Panda device's firmware if you're running into issues."));
  QObject::connect(flashPandaBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to flash the Panda?"), tr("Flash"), this)) {
      std::thread([=]() {
        uiState()->scene.keep_screen_on = true;

        flashPandaBtn->setEnabled(false);
        flashPandaBtn->setValue(tr("Flashing..."));

        system("python3 /data/openpilot/panda/board/flash.py");
        system("python3 /data/openpilot/panda/board/recover.py");
        system("python3 /data/openpilot/panda/tests/reflash_internal_panda.py");

        flashPandaBtn->setValue(tr("Flashed!"));
        util::sleep_for(2000);
        flashPandaBtn->setValue(tr("Rebooting..."));
        util::sleep_for(2000);
        Hardware::reboot();
      }).detach();
    }
  });
  addItem(flashPandaBtn);

  forceStartedBtn = new FrogPilotButtonsControl(tr("Force Started State"), tr("Forces openpilot either offroad or onroad."), {tr("OFFROAD"), tr("ONROAD"), tr("OFF")}, true);
  QObject::connect(forceStartedBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
    if (id == 0) {
      paramsMemory.putBool("ForceOffroad", true);
      paramsMemory.putBool("ForceOnroad", false);
    } else if (id == 1) {
      paramsMemory.putBool("ForceOffroad", false);
      paramsMemory.putBool("ForceOnroad", true);
    } else if (id == 2) {
      paramsMemory.putBool("ForceOffroad", false);
      paramsMemory.putBool("ForceOnroad", false);
    }
    forceStartedBtn->setCheckedButton(id);
  });
  forceStartedBtn->setCheckedButton(2);
  addItem(forceStartedBtn);

  ButtonControl *resetTogglesBtn = new ButtonControl(tr("Reset Toggles to Default"), tr("RESET"), tr("Resets your toggle settings back to their default settings."));
  QObject::connect(resetTogglesBtn, &ButtonControl::clicked, [=]() mutable {
    QDir toggleDir("/data/params/d");

    if (ConfirmationDialog::confirm(tr("Are you sure you want to completely reset all of your toggle settings?"), tr("Reset"), this)) {
      std::thread([=]() mutable {
        resetTogglesBtn->setEnabled(false);
        resetTogglesBtn->setValue(tr("Resetting..."));

        if (toggleDir.removeRecursively()) {
          toggleDir.mkpath(".");
          params.putBool("DoToggleReset", true);

          resetTogglesBtn->setValue(tr("Reset!"));

          util::sleep_for(2000);
          resetTogglesBtn->setValue(tr("Rebooting..."));
          util::sleep_for(2000);

          Hardware::reboot();
        } else {
          resetTogglesBtn->setValue(tr("Failed..."));
        }
      }).detach();
    }
  });
  addItem(resetTogglesBtn);
}
