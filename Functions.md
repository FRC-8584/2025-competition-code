# 2025 Robot Functions

# Parts

## Claw and Elevator
- Grab CORAL (SOURCE) 
- Grab ALGAE (REEF) 
- Put CORAL (REEF) 
- Put ALGAE (PROCESSOR) 
## Intake
- Grab ALGAE (ground) 
- Put ALGAE (PROCESSOR) 
## Swerve
- Move with limelight (aim REEF) 

# Combination Actions
- Move swerve to the front of the REEF and put CORAL automaticly (left or right) 
- Move swerve to the front of the REEF and grab ALGAE automaticly (left or right)
- Grab CORAL till get (SOURCE) 
- To different states (8 levels with two different default state)

# To be determined
- Auto with PathPlanner
- Move swerve to the front of the PROCESSOR and put ALGAE

# Keys

## Player 1

### Axies
| Axie ID | Action |
| -------- | -------- |
|0|Swerve Y drive speed|
|1|Swerve X drive speed|
|3|Swerve turn drive speed|
|5|Grab Algae by claw|
|6|Put Algae by claw|

### Buttons
| Button ID | Action |
| -------- | -------- |
|2|Aim right reef|
|3|Aim left reef|
|4|Aim middle reef|
|5|Grab coral| 
|6|Put coral|
|8|Change drive method (with or without gyro)|

## Player 2

### Axies
| Axie ID | Action |
| -------- | -------- |
|3|Intake(intake)|
|4|Outake(intake)|

### Buttons
| Button ID | Action | 
| -------- | -------- |
|1|To default (level 1)|
|2|To coral level 2|
|3|To coral level 4|
|4|To coral level 3|
|5 + Hat-Up|To algae level 2|
|5 + Hat-Down|To algae level 1|
|6|To default with Algae|

![image](https://hackmd.io/_uploads/SkBSiHCoJx.png)