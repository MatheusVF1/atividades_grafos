import json

#Ler o arquivo json
with open('cities.json') as json_cidades:
    cidades = json.load(json_cidades)

list_latitude = []
list_longitude = []
list_nomes = []

for i in cidades:
    list_latitude.append(i['latitude'])
    list_longitude.append(i['longitude'])
    list_nomes.append(i['city'])

print(list_nomes[0])