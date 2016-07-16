var ws = require('ws').Server;

var wss = new ws({
    host: 'localhost',
    port: 3000
});

console.log("START!");

wss.broadcast = function (data) {
    for (var i in this.clients) {
        this.clients [i].send(data);
    }
};
wss.on('open', function () {
    ws.send('Hello WebSocket');
});
wss.on('connection', function (ws) {
    ws.on('message', function (message) {
        var now = new Date();
        console.log('Received: %s', message);
        wss.broadcast(message);
    });
});