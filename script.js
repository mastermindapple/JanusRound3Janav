/* For arduino port
let port;
let reader;
let keepReading = true;

// Plot data arrays
let timeData = [];
let sensorData = [];

// 3D data arrays for latitude, longitude, altitude
let latData = [];
let lonData = [];
let altData = [];

document.getElementById('connectButton').addEventListener('click', async () => {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 9600 });
        readSerial();
    } catch (err) {
        console.error('Error connecting to serial port:', err);
    }
});

async function readSerial() {
    const decoder = new TextDecoderStream();
    const inputDone = port.readable.pipeTo(decoder.writable);
    reader = decoder.readable.getReader();

    let t = 0;

    while (keepReading) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) {
            // Expecting CSV like: sensor,lat,lon,alt
            const parts = value.trim().split(',');
            if (parts.length >= 1) {
                // Sensor value
                const sensor = parseFloat(parts[0]);
                timeData.push(t);
                sensorData.push(sensor);
                t++;

                // Update 2D plot
                Plotly.newPlot('plot', [{
                    x: timeData,
                    y: sensorData,
                    mode: 'lines+markers',
                    name: 'Sensor'
                }], {
                    title: 'Live Sensor Data',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Sensor Value' }
                });
            }

            if (parts.length >= 4) {
                // Lat, Lon, Alt
                const lat = parseFloat(parts[1]);
                const lon = parseFloat(parts[2]);
                const alt = parseFloat(parts[3]);

                latData.push(lat);
                lonData.push(lon);
                altData.push(alt);

                // Update 3D plot
                Plotly.newPlot('plot3D', [{
                    x: lonData,
                    y: latData,
                    z: altData,
                    mode: 'lines+markers',
                    type: 'scatter3d'
                }], {
                    title: '3D GPS Plot',
                    scene: {
                        xaxis: { title: 'Longitude' },
                        yaxis: { title: 'Latitude' },
                        zaxis: { title: 'Altitude' }
                    }
                });
            }
        }
    }
}
    */

//for csv


const csvDataText = `
512,37.7749,-122.4194,30
523,37.7750,-122.4195,32
498,37.7751,-122.4196,31
505,37.7752,-122.4197,29
510,37.7753,-122.4198,30
`;

const csvData = csvDataText.trim().split('\n').map(line => line.split(','));


let timeData = [];
let sensorData = [];

let latData = [];
let lonData = [];
let altData = [];

let simulationIndex = 0;



document.getElementById('startButton').addEventListener('click', () => {
    if (csvData.length === 0) {
        alert("CSV data not loaded yet!");
        return;
    }
    simulateData();
});

function simulateData() {
    const interval = setInterval(() => {
        if (simulationIndex >= csvData.length) {
            clearInterval(interval);
            return;
        }

        const row = csvData[simulationIndex];
        const sensor = parseFloat(row[0]);
        const lat = parseFloat(row[1]);
        const lon = parseFloat(row[2]);
        const alt = parseFloat(row[3]);

        timeData.push(simulationIndex);
        sensorData.push(sensor);

        latData.push(lat);
        lonData.push(lon);
        altData.push(alt);

        // Update 2D sensor plot
        Plotly.newPlot('plot', [{
            x: timeData,
            y: sensorData,
            mode: 'lines+markers',
            name: 'Sensor'
        }], {
            title: 'Sensor Data',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Sensor Value' }
        });

        // Update 3D plot
        Plotly.newPlot('plot3D', [{
            x: lonData,
            y: latData,
            z: altData,
            mode: 'lines+markers',
            type: 'scatter3d'
        }], {
            title: '3D GPS Plot',
            scene: {
                xaxis: { title: 'Longitude' },
                yaxis: { title: 'Latitude' },
                zaxis: { title: 'Altitude' }
            }
        });

        simulationIndex++;
    }, 500); // Adjust speed (ms) here
}

