#include <Arduino_KNN.h>

KNNClassifier classifier(2);

String clsName[] = {"open", "close"};

void train() {
  int cls = Serial.readString().toInt();
  Serial.print(clsName[cls]);
  Serial.print(" class 5s recording started... ");
  for (int i=0; i<500; i++) {
    float emg[] = { (float)analogRead(34), (float)analogRead(35) };
    classifier.addExample(emg, cls);
    delay(10);
  }
  Serial.print("done. total count = ");
  Serial.println(classifier.getCount());
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) train();
  
  float emg[] = { (float)analogRead(34), (float)analogRead(35) };
  if (classifier.getCount() >= 1000) {
    Serial.print(clsName[classifier.classify(emg, 100)]);
    Serial.print(" ");
    Serial.println(classifier.confidence());
  }
  delay(100);
}
