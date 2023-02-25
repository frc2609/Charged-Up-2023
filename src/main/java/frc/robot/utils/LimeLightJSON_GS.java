package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/** DO NOT MODIFY THIS CLASS
 * This class is required by the Jackson JSON parsing api.
*/
public class LimeLightJSON_GS {
  @JsonProperty
  Results Results;

  // Getter Methods 

  public Results getResults() {
    return Results;
  }

  // Setter Methods 

  public void setResults(Results Results) {
    this.Results = Results;
  }
}
