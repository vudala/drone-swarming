import airsim


WIND_NORTH_SPEED_MS = 0
WIND_EAST_SPEED_MS = 0
WIND_DOWN_SPEED_MS = 0

# 0.0 - 1.0
RAIN_INTENSITY = 0.0

# 0.0 - 1.0
DUST_CONCENTRATION = 0.0

# 0.0 - 1.0
FOG_CONCENTRATION = 0.0

# 0.0 - 1.0
SNOW_CONCENTRATION = 0.0

# 0.0 - 1.0
MAPLE_LEAF_CONCENTRATION = 0.5


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

# fog concentration
client.simSetWeatherParameter(
    airsim.WeatherParameter.Fog,
    FOG_CONCENTRATION
)

# snow concentration
client.simSetWeatherParameter(
    airsim.WeatherParameter.Snow,
    SNOW_CONCENTRATION
)

# maple leaf concentration
client.simSetWeatherParameter(
    airsim.WeatherParameter.MapleLeaf,
    MAPLE_LEAF_CONCENTRATION
)

input('Barrier')

client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0)
client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, 0)

client.simEnableWeather(False)
