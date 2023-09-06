import airsim


WIND_NORTH_SPEED_MS = 0
WIND_EAST_SPEED_MS = 0
WIND_DOWN_SPEED_MS = 8

# 0.0 - 1.0
RAIN_INTENSITY = 0

# 0.0 - 1.0
DUST_CONCENTRATION = 0


client = airsim.MultirotorClient()
client.simEnableWeather(True)

# wind NED
wind = airsim.Vector3r(
    WIND_NORTH_SPEED_MS,
    WIND_EAST_SPEED_MS,
    WIND_DOWN_SPEED_MS
)
client.simSetWind(wind)

# rain intensity
client.simSetWeatherParameter(
    airsim.WeatherParameter.Rain,
    RAIN_INTENSITY
)

# dust concentration
client.simSetWeatherParameter(
    airsim.WeatherParameter.Dust,
    DUST_CONCENTRATION
)

input('Barrier')

client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0)

client.simEnableWeather(False)
