import pandas as pd
import folium

csv_file = 'gps_data_driving.csv'
df = pd.read_csv(csv_file)

latitudes = df['latitude']
longitudes = df['longitude']
center_lat = latitudes.mean()
center_lon = longitudes.mean()
mymap = folium.Map(location=[center_lat, center_lon], zoom_start=15)

for lat, lon in zip(latitudes, longitudes):
    folium.CircleMarker(location=[lat, lon],
                        radius=2,
                        color='blue',
                        fill=True,
                        fill_opacity=0.7).add_to(mymap)
    
mymap.save("gps_map1.html")