void dumpfloat(float number) {
  char* cursor = (char*)&number;

  Serial.print(number);
  Serial.print("  = [");
  Serial.print((int)*cursor, HEX);
  Serial.print(" ");

  Serial.print((int)*(cursor+1), HEX);
  Serial.print(" ");

  Serial.print((int)*(cursor+2), HEX);
  Serial.print(" ");
 
  Serial.print((int)*(cursor+3), HEX);
  Serial.print(" ");

  Serial.println("]" );

}
