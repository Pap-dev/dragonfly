"""
Wind Speed and Direction: Knowing the wind speed and direction is essential for safe drone operation. Strong winds can push the drone off course and cause instability, which may result in a crash.
Temperature: Drones have specific operating temperature ranges, and extreme temperatures can impact battery performance and affect the drone's flight capabilities.
Humidity: High humidity can cause moisture to build up on the drone's electrical components, leading to malfunctioning or even failure.
Precipitation: Rain, snow, or other forms of precipitation can damage the drone and interfere with the camera's operation, leading to unsafe flying conditions.
Visibility: Good visibility is crucial when operating a drone. Low visibility conditions, such as fog, can impair the pilot's ability to navigate and avoid obstacles.
Air Pressure: Changes in air pressure can affect a drone's altitude and stability, so it is essential to be aware of the current air pressure in the area where you are flying.
Obstacles: Before flying, it is crucial to identify potential obstacles in the area, such as buildings, trees, or power lines, to avoid collisions.
"""
import geocoder
import requests
import socket
import geocoder
import json
from geopy.geocoders import Nominatim

# AIRMAP_API_KEY = ""
# OPEN_WATHER_API_KEY = ""

@staticmethod
def check_internet_connection():
    try:
        socket.create_connection(("8.8.8.8", 53))
        return True
    except OSError:
        pass
    return False

def atmospheric_conditions(latitude = str, longitude = str) -> dict:
    """
    Returns the atmospheric conditions at the UAV's current location.
    """
    conditions = {}

    url = f"http://api.openweathermap.org/data/2.5/weather?lat={latitude}&lon={longitude}&appid={OPEN_WATHER_API_KEY}&units=metric"
    response = requests.get(url)
    data = response.json()

    conditions["temperature"] = data['main']['temp']
    conditions["humidity"] = data['main']['humidity']
    conditions["wind_speed"] = data['wind']['speed']
    conditions["wind_direction"] = data['wind']['deg']
    conditions["precipitation"] = data['weather'][0]['description']
    conditions["visibility"] = data['visibility']
    conditions["air_pressure"] = data['main']['pressure']

    return conditions

def get_flight_restrictions(latitude, longitude):
    """
    Returns the flight restrictions at the UAV's current location.
    """

    # Make the request to the AirMap API
    response = requests.get(
        f'https://api.airmap.com/airspaces/v2/nearest?latitude={latitude}&longitude={longitude}&buffer=500',
        headers={'X-API-Key': AIRMAP_API_KEY}
    )

    # Parse the response
    if response.status_code == 200:
        data = response.json()
        if data['status'] == 'success' and data['data']:
            airspace = data['data'][0]
            if airspace['type'] == 'No Fly Zone':
                print('You are in a No Fly Zone')
            else:
                print('You are not in a No Fly Zone')
        else:
            print('No airspace data available')
    else:
        print(f'Request failed with status code {response.status_code}')

@staticmethod
def get_gps_location():
    latitude_longitude = geocoder.ip('me').latlng 
    return latitude_longitude

def main():
    """
    Main function.
    """
    internet_connection = check_internet_connection()
    if internet_connection:
        latitude_longitude = get_gps_location()
        latitude = latitude_longitude[0]
        longitude = latitude_longitude[1]

        print("Current location : ")
        print(f"    - latitude  : " + str(latitude))
        print(f"    - longitude : " + str(longitude))

        conditions = atmospheric_conditions(latitude, longitude)
        flight_restriction = get_flight_restrictions(latitude, longitude)
        print(conditions)
        print(flight_restriction)

if __name__ == "__main__":
    main()
