import QtQuick 2.5
import QtQuick.Window 2.1
import QtQuick.Controls 1.4
import QtGamepad 1.0

Window {

    Connections {
        target: GamepadManager
        onGamepadConnected: gamepad.deviceId = deviceId
        onButtonConfigured: pressButton(null)
        onAxisConfigured: pressButton(null)
        onConfigurationCanceled: pressButton(null)
    }

    Gamepad {
        id: gamepad
        deviceId: GamepadManager.connectedGamepads.length > 0 ? GamepadManager.connectedGamepads[0] : -1
    }

    Item {
        property double buttonL1: gamepad.buttonL1
        onButtonL1Changed: { mController.setL1(gamepad.buttonL1);
        }
        property double buttonR1: gamepad.buttonR1
        onButtonR1Changed: { mController.setR1(gamepad.buttonR1);
        }
        property double buttonL2: gamepad.buttonL2
        onButtonL2Changed: { mController.setL2(gamepad.buttonL2);
        }
        property double buttonR2: gamepad.buttonR2
        onButtonR2Changed: { mController.setR2(gamepad.buttonR2);
        }
        property double axisLeftX: gamepad.axisLeftX
        onAxisLeftXChanged: { mController.setAxisLeftX(gamepad.axisLeftX);
        }
        property double axisLeftY: gamepad.axisLeftY
        onAxisLeftYChanged: { mController.setAxisLeftY(gamepad.axisLeftY);
        }
        property double buttonY: gamepad.buttonY
        onButtonYChanged: { mController.setY(gamepad.buttonY);
        }
//        property double buttonB: gamepad.buttonB
//        onButtonBChanged: { mController.setB(gamepad.buttonB);
//        }
//        property double buttonA: gamepad.buttonA
//        onButtonAChanged: { mController.setA(gamepad.buttonA);
//        }
//        property double buttonUp: gamepad.buttonUp
//        onButtonUpChanged: { mController.setUp(gamepad.buttonUp);
//        }
//        property double buttonDown: gamepad.buttonDown
//        onButtonDownChanged: { mController.setDown(gamepad.buttonDown);
//        }
//        property double buttonLeft: gamepad.buttonLeft
//        onButtonLeftChanged: { mController.setLeft(gamepad.buttonLeft);
//        }
//        property double buttonRight: gamepad.buttonRight
//        onButtonRightChanged: { mController.setRight(gamepad.buttonRight);
//        }
//        property double buttonStart: gamepad.buttonStart
//        onButtonStartChanged: { mController.setStart(gamepad.buttonStart);
//        }
    }
}
