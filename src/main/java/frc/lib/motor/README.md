# Motor
## Setup
in `robotPeriodic` add
```
MotorFactory.get().MotorFactory.get().updateSims();
```
## Usage
### New Motor
```java
Motor motorName = MotorFactory.get().create(motorDefinition);
```
