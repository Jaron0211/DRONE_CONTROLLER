using namespace RC;


void CH_PRINT(){
  for(int i=0;i<=15;i++){
    Serial.print(CH[i]);
    Serial.print(" , ");
  }
  Serial.println();
}
