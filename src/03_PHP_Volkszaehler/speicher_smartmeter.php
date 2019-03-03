<?php

$USERNAME = "******";   // Für Basic Authentication
$PASSWORD = "******";
$APP_ID  = "*****";     // App ID from TTN

$url_middleware = "https://  ******* /middleware.php/data/";

$uuid_spannung = "*******";
$uuid_energie  = "*******";
$uuid_leistung = "*******";

$rawdata = file_get_contents('php://input');

#
# Read JSON
#

$json = json_decode($rawdata,true);

if ($json['app_id'] != $APP_ID) {
	fwrite(STDERR, "App\n");
	exit(1); // A response code other than 0 is a failure
}
$time     = $json['metadata']['time'];
$power    = $json['payload_fields']['P'];
$energy   = $json['payload_fields']['W'];
$voltage  = $json['payload_fields']['V'];

$ts	= 1000 * strtotime( substr($time, 0, 23) . 'Z' );

# 
# DEBUG
#

#file_put_contents('./log.log', ("Time = ".$time." P = ".$power."\n"), FILE_APPEND);


#
# Save data
#

$ch = curl_init();
curl_setopt($ch, CURLOPT_HEADER, 1);
curl_setopt($ch, CURLOPT_USERPWD, $USERNAME . ":" . $PASSWORD);
curl_setopt($ch, CURLOPT_TIMEOUT, 30);
curl_setopt($ch, CURLOPT_POST, 1);


# Speicher Spannung
if ($voltage > 0) {
	$url = $url_middleware . $uuid_spannung . ".json?operation=add&ts=". $ts . "&value=" . $voltage/1000;
	curl_setopt($ch, CURLOPT_URL, $url);
	$output = curl_exec($ch);
}

# Speicher Leistung
if ($power > 0) {
	$url = $url_middleware . $uuid_leistung . ".json?operation=add&ts=". $ts . "&value=" . $power;
	curl_setopt($ch, CURLOPT_URL, $url);
	$output = curl_exec($ch);
}

# Speicher Energie
if ($energy > 0) {
	$url = $url_middleware . $uuid_energie . ".json?operation=add&ts=". $ts . "&value=" . $energy;
	curl_setopt($ch, CURLOPT_URL, $url);
	$output = curl_exec($ch);
}

curl_close($ch);

echo "<pre>Success</pre>";


?>