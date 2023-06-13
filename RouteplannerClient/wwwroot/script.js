
mapboxgl.accessToken = '';
const map = new mapboxgl.Map({
    container: 'map', // container ID
    // Choose from Mapbox's core styles, or make your own style with Mapbox Studio
    style: 'mapbox://styles/mapbox/streets-v12', // style URL
    center: [9.554, 47.15], // starting position [lng, lat]
    zoom: 10 // starting zoom
});

map.on('load', () => {
    map.addSource('route', {
        'type': 'geojson',
        'data': 'results/result.json'
    });
    map.addLayer({
        'id': 'route',
        'type': 'line',
        'source': 'route',
        'layout': {
            'line-join': 'round',
            'line-cap': 'round'
        },
        'paint': {
            'line-color': '#ff06db',
            'line-width': 4
        }
    });
});