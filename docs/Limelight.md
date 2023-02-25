# Limelight

(how do you use the Limelight?)

- When AprilTag is out of view, Limelight::tags.get(id) will be null.
  > One of the limelight files should handle this.

- When previously in-view AprilTag goes out of view, Limelight::tags.get(id) will have the same value.
  > This should not occur.