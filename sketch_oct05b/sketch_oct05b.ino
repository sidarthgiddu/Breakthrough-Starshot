static const char b64chars[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

String Hash_base64( uint8_t *in, int hashlength) {
  int i, out;
  char b64[80]; // working byte array for sextets....
  String base64;
  for (i = 0, out = 0 ;; in += 3) { // octets to sextets
    i++;
    b64[out++] = b64chars[in[0] >> 2];

    if (i >= hashlength ) { // single byte, so pad two times
      b64[out++] = b64chars[((in[0] & 0x03) << 4) ];
      b64[out++] =  '=';
      b64[out++] =  '=';
      break;
    }

    b64[out++] = b64chars[((in[0] & 0x03) << 4) | (in[1] >> 4)];
    i++;
    if (i >= hashlength ) { // two bytes, so we need to pad one time;
      b64[out++] =  b64chars[((in[1] & 0x0f) << 2)] ;
      b64[out++] =  '=';
      break;
    }
    b64[out++] = b64chars[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
    b64[out++] =   b64chars[in[2] & 0x3f];

    i++;
    if (i >= hashlength ) { // three bytes, so we need no pad - wrap it;
      break;
    }
  } // this should make b64 an array of sextets that is "out" in length
  b64[out] = 0;
  base64 = b64;
  return (base64);
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  delay(1000);

}
void loop() {
  uint8_t in[10] = {1,2,3,4,5,6,7,8,9,0};
  Serial.println(Hash_base64( in, 10));
  delay(1000);
}

