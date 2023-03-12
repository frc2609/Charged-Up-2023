# Limelight

## Web Dashboard
hi

## Camera View
(Instructions are for Shuffleboard.)
- From `Sources`, select `CameraServer`
- Drag the `limelight` entry onto Shuffleboard and resize/move it as necessary

## Programming
- When AprilTag is out of view, Limelight::tags.get(id) will be null.
  > One of the limelight files should handle this.

- When previously in-view AprilTag goes out of view, Limelight::tags.get(id) will have the same value.
  > This should not occur.