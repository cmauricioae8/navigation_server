var queryStringParams = (function(a) {
    if (a == "") return {};
    var b = {};
    for (var i = 0; i < a.length; ++i)
    {
        var p=a[i].split('=', 2);
        if (p.length == 1)
            b[p[0]] = "";
        else
            b[p[0]] = decodeURIComponent(p[1].replace(/\+/g, " "));
    }
    return b;
})(window.location.search.substr(1).split('&'));


var urlServer = queryStringParams.host || document.location.hostname+':'+document.location.port;

console.log("Connecting to SocketIO server on:", urlServer);

var socketio = io(urlServer);

// funcion para atrapar mensajes durante la conexion
socketio.on("connect", () => {
    console.log("connection success"); // prints the message associated with the message
});
// funcion para atrapar errores del servidor durante la conexion
socketio.on("connect_error", (err) => {
    console.log(err.msg); // prints the message associated with the error
});
// funcion para atrapar errores del servidor
socketio.on("errors", (err) => {
    console.log(err.msg); // prints the message associated with the error
});
