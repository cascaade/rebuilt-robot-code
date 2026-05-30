# 2026 rebuilt state machine breakdown

tracking all states across all subsystems

some of the file names are not as they seem because there's already a file named _Subsystem from the previous code version

## a little bit of a note

these states should cover <u>all</u> wanted outcomes from the robot, so if there is something you want to command the robot to do, it has to fit into one of the states provided

## Climb - ClimberSubsystem.java

| WantedState      | description |   | SystemState        | description |
|------------------|-------------|---|--------------------|-------------|
| IDLE             |             |   | IDLING             |             |
| HOME             |             |   | HOMING             |             |
| STOW             |             |   | STOWED             |             |
| MOVE_TO_POSITION |             |   | MOVING_TO_POSITION |             |
| WINCH_READY      |             |   | READYING           |             |
| MANUAL_CLIMB     |             |   | MANUAL_CLIMBING    |             |
| LOCKED           |             |   | LOCKED             |             |

## Indexer - IndexerSubsystem.java

| WantedState | description |   | SystemState | description |
|-------------|-------------|---|-------------|-------------|
| IDLE        |             |   | IDLING      |             |
| FEED        |             |   | FEEDING     |             |
| REVERSE     |             |   | REVERSING   |             |

## Intake - IntakeFSM.java

| WantedState | description |   | SystemState  | description |
|-------------|-------------|---|--------------|-------------|
| IDLE        |             |   | IDLING       |             |
| HOME        |             |   | HOMING_WRIST |             |
| INTAKE      |             |   | INTAKING     |             |
| OUTTAKE     |             |   | OUTTAKING    |             |
| PULSE       |             |   | PULSING      |             |
| STOW        |             |   | STOWING      |             |
|             |             |   | DEPLOYING    |             |

### Rollers - Rollers.java

| WantedState      | description |   | SystemState        | description |
|------------------|-------------|---|--------------------|-------------|
| IDLE             |             |   | IDLING             |             |
| INTAKE           |             |   | INTAKING           |             |
| OUTTAKE          |             |   | OUTTAKING          |             |

### Wrist - Wrist.java

| WantedState  | description |   | SystemState     | description |
|--------------|-------------|---|-----------------|-------------|
| IDLE         |             |   | IDLING          |             |
| STOW         |             |   | STOWING         |             |
| DEPLOY       |             |   | DEPLOYING       |             |
| HOME         |             |   | HOMING          |             |

## LEDs - LEDSubsystem.java

| WantedState  | description |   | SystemState  | description |
|--------------|-------------|---|--------------|-------------|
| DISPLAY_OFF  |             |   | DISPLAY_OFF  |             |
| DISABLED     |             |   | DISABLED     |             |
| DISCONNECTED |             |   | DISCONNECTED |             |
| AUTONOMOUS   |             |   | AUTONOMOUS   |             |
| ENABLED      |             |   | ENABLED      |             |
| BOOT         |             |   | BOOT         |             |

## Shooter - ShooterFSM.java

| WantedState      | description |   | SystemState        | description |
|------------------|-------------|---|--------------------|-------------|
| IDLE             |             |   | IDLING             |             |
| RESPONSIVE       |             |   | RESPONDING         |             |
| PASS             |             |   | PASSING            |             |

## Swerve - SwerveFSM.java

| WantedState | description |   | SystemState | description |
|-------------|-------------|---|-------------|-------------|
| STOP        |             |   | STOPPED     |             |
| SYS_ID      |             |   | SYS_ID      |             |
| AIM_HUB     |             |   | AIMING_HUB  |             |
| AIM_PASS    |             |   | AIMING_PASS |             |
| TELEOP      |             |   | TELEOP      |             |
| TRAJECTORY  |             |   | TRAJECTORY  |             |
| CROSS       |             |   | CROSSED     |             |

## Superstructure - Superstructure.java

| WantedState      | description |   | SystemState      | description |
|------------------|-------------|---|------------------|-------------|
| DEFAULT          |             |   | DEFAULT          |             |
| AUTO             |             |   | AUTO             |             |
| TELEOP           |             |   | TELEOP           |             |
| INTAKE_TELEOP    |             |   | INTAKE_TELEOP    |             |
| AIM_TELEOP       |             |   | AIMING_TELEOP    |             |
| SHOOT_TELEOP     |             |   | SHOOTING_TELEOP  |             |
| PROTECTED_TELEOP |             |   | PROTECTED_TELEOP |             |
