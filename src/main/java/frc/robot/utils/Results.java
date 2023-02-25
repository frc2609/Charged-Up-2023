package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;

import com.fasterxml.jackson.annotation.JsonProperty;

/** DO NOT MODIFY THIS CLASS
 * This class is required by the Jackson JSON parsing api.
*/
public class Results {
  @JsonProperty
  public ArrayList < Object > Classifier = new ArrayList < Object > ();
  @JsonProperty
  public ArrayList < Object > Detector = new ArrayList < Object > ();
  @JsonProperty
  public ArrayList < HashMap<String, Object> > Fiducial = new ArrayList < HashMap<String, Object>> ();
  @JsonProperty
  public ArrayList < Object > Retro = new ArrayList < Object > ();
  @JsonProperty
  private float pID;
  private float tl;
  private float ts;
  private float v;
  private float[] botpose;
  private float[] botpose_wpiblue;
  private float[] botpose_wpired;

  // Getter Methods 

  public ArrayList< Object > getClassifier() {
    return Classifier;
  }

  public ArrayList< Object > getDetector() {
    return Detector;
  }

  public ArrayList< HashMap<String, Object> > getFiducial() {
    return Fiducial;
  }

  public ArrayList< Object > getRetro() {
    return Retro;
  }

  public float getPID() {
    return pID;
  }

  public float getTl() {
    return tl;
  }

  public float getTs() {
    return ts;
  }

  public float getV() {
    return v;
  }

  public float[] getBotpose() {
    return botpose;
  }

  public float[] getBotpose_wpiblue() {
    return botpose_wpiblue;
  }

  public float[] getBotpose_wpired() {
    return botpose_wpired;
  }

  // Setter Methods 

  public void getClassifier(ArrayList< Object > Classifier) {
    this.Classifier = Classifier;
  }

  public void getDetector(ArrayList< Object > Detector) {
    this.Detector = Detector;
  }

  public void getFiducial(ArrayList< HashMap<String, Object> > Fiducial) {
    this.Fiducial = Fiducial;
  }

  public void getRetro(ArrayList< Object > Retro) {
    this.Retro = Retro;
  }

  public void setPID(float pID) {
    this.pID = pID;
  }

  public void setTl(float tl) {
    this.tl = tl;
  }

  public void setTs(float ts) {
    this.ts = ts;
  }

  public void setV(float v) {
    this.v = v;
  }

  public void setBotpose(float[] botpose) {
    this.botpose = botpose;
  }

  public void setBotpose_wpiblue(float[] botpose_wpiblue) {
    this.botpose_wpiblue = botpose_wpiblue;
  }

  public void setBotpose_wpired(float[] botpose_wpired) {
    this.botpose_wpired = botpose_wpired;
  }

}
