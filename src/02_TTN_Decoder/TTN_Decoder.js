function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var energiemenge = 0;
  for (var i = 0; i < 8; i++) {
    energiemenge += bytes[i] * Math.pow(2, 8 * (7-i)) ;
  }
  energiemenge = energiemenge/10000

  var leistung = 0;
  for (i = 0; i < 4; i++) {
    leistung += bytes[i+8] * Math.pow(2, 8 * (3-i))  ;
  }
  leistung = Math.round(leistung*100)/100
  
  var spannung = 0;
  for (i = 0; i < 2; i++) {
    spannung += bytes[i+12] * Math.pow(2, 8 * (1-i)) ;
  }
  spannung = spannung;

  return {V: spannung, W: energiemenge, P:leistung};
}