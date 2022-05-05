let id_input = document.querySelector('#id-input');
let load_btn = document.querySelector('#load-btn');
let data = null;

function load_session(save_id=true) {
    let session_id = id_input.value;
    console.log('Selected session ID:', session_id);
    let url = `ws://${location.host}/ws?session_id=${session_id}`;
    let socket = new WebSocket(url);
    socket.onmessage = (e) => {
        data = JSON.parse(e.data);
        socket.close();
        console.log('Session data loaded:', data);
        if (save_id) localStorage.setItem('session_id', session_id);
        build_plot();
    };
}

// function make_default_groups() {
//     return Object.values(data.values).map(value => ({
//         name: `${value.name} [default group]`,
//         values: [value.id],
//     }));
// }

function make_default_groups() {
    return [
        {values: ['state.pole_angle', 'state.pole_angular_velocity'], name: 'KOK'},
        {values: ['state.position', 'state.velocity'], name: 'KEK'},
    ]
}

function make_traces() {
    let traces = [];
    for (let [index, group] of data.groups.entries()) {
        for (let value_id of group.values) {
            let value_data = data.values[value_id];
            let trace = {
                type: "scattergl",
                mode: "lines+markers",
                name: value_data.name,
                x: value_data.x,
                y: value_data.y,
                xaxis: "x",
                yaxis: "y" + (index ? index + 1 : ''),
                legendgroup: "group" + (index ? index + 1 : ''),
                legendgrouptitle: {text: group.name},
            }
            traces.push(trace);
            console.log(trace);
        }
    }
    return traces;
}

function make_layout() {
    return {
        height: data.groups.length * 500,
        autosize: true,
        grid: {
            rows: data.groups.length,
            columns: 1,
            subplots: data.groups.map((_, i) => ['xy' + (i ? i + 1 : '')]),
            ygap: 0.05,
        },
        legend: {
            orientation: 'h',
            yanchor: 'top',
            xanchor: 'center',
            x: 0.5,
            y: 1.2,
        }
    };
}

function build_plot() {
    if (data.groups.length === 0) data.groups = make_default_groups();
    let config = {responsive: true};
    Plotly.newPlot("chart", make_traces(), make_layout(), config);
}

function init() {
    let params = new URLSearchParams(location.search);
    if (params.get('id')) {
        id_input.value = params.get('id');
        load_session(false);
        document.querySelector('#toolbar').style.display = 'none';
    } else {
        id_input.value = localStorage.getItem('session_id');
    }
}

load_btn.addEventListener('click', load_session);
id_input.addEventListener('keyup', e => e.keyCode === 13 ? load_session() : null);
window.addEventListener('load', init);
