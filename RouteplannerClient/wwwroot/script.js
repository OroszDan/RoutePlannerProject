
mapboxgl.accessToken = 'pk.eyJ1Ijoib3Jvc3pkYW4iLCJhIjoiY2xpcmo4NDFiMGI3YTNyb3RkNTl2Z3YydiJ9.vClhA5hrzWzQrv28QGUc-Q';
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

let currentMarkers = [];

map.on('click', (e) => {

    console.log('x coordinate: ' + e.lngLat.lng)
    console.log('y coordinate: ' + e.lngLat.lat)

    if (currentMarkers.length < 2) {
        let marker = new mapboxgl.Marker()
            .setLngLat([e.lngLat.lng, e.lngLat.lat])
            .addTo(map);

        //if (currentMarkers.length == 0) {
        //    marker.getElement().setAttribute('id', 'startMarker')
        //}
        //else {
        //    marker.getElement().setAttribute('id', 'finishMarker')
        //}


        marker.getElement().addEventListener('click', (e) => {
            e.stopPropagation();

            let index = currentMarkers.indexOf(marker)
            currentMarkers.splice(index, 1)
            marker.remove()
            
        });

        currentMarkers.push(marker)
    } 
});

document.querySelector('#startButton').addEventListener('click', startRoutePlanning)

getCars()

function getCars() {

    fetch('http://localhost:18080/getcars', {
        method: "GET",
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json));;
}

function createOptions(json) {
    const carValues = ['peugeot_208', 'bmw_3', 'mercedes_c']
    const carTexts = ['Peugeot 208', 'BMW 3 Series', 'Mercedes C Class']
    let carSelect = document.querySelector('#carSelect')

    for (var i = 0; i < carValues.length; i++) {
        let option = document.createElement('option')
        option.value = carValues[i]
        option.innerHTML = carTexts[i]
        carSelect.appendChild(option)
    }
}

function startRoutePlanning() {
    if (currentMarkers.length == 2) {
        console.log('Starting search')

        let carSelect = document.querySelector('#carSelect')
        let selectedCarValue = carSelect.options[carSelect.selectedIndex].value
        console.log('Selected car: ' + selectedCarValue)

        const sendData = {
            startLon: currentMarkers[0]._lngLat.lng,
            startLat: currentMarkers[0]._lngLat.lat,
            finishLon: currentMarkers[1]._lngLat.lng,
            finishLat: currentMarkers[1]._lngLat.lat,
            carModel: selectedCarValue
        }

        console.log(sendData)

        fetch('http://localhost:18080/search', {
            method: "POST",
            body: JSON.stringify(sendData),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .then((response) => response.json())
        .then((json) => console.log(json));;
    }
    else {
        alert('No start or finish selected!')
    }
}