import urllib.request
import re

url = "https://raw.githubusercontent.com/adafruit/Adafruit_BMP3XX/master/Adafruit_BMP3XX.cpp"
content = urllib.request.urlopen(url).read().decode('utf-8')

start = content.find("float Adafruit_BMP3XX::compensate_temp")
end = content.find("float Adafruit_BMP3XX::compensate_pressure", start) + 1000

print(content[start:end])
