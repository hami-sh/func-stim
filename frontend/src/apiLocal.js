const io = require("socket.io")(8000, {
    cors: {
        origin: "http://localhost:3000",
        methods: ["GET", "POST"],
        allowedHeaders: ["my-custom-header"],
        credentials: true
    }
});
// const socketIOClient = require("socket.io-client");
console.log("started");

// const socket = socketIOClient(ENDPOINT);

io.on("connection", socket => {
    // handle the event sent with socket.send()
    console.log("New client connected");

    socket.onAny((event, ...args) => {
        console.log(`event: ${event} | args: ${args}`);
        if (event !== "connect") {
            console.log("sending...");
            io.sockets.emit(event, args);
        }
    });

    let change_mode = {
        mode: 0
    }
    // socket.emit("stimulator_change_mode", JSON.stringify(change_mode));
    //
    // change_mode = {
    //     mode: 1
    // }
    // socket.emit("stimulator_change_mode", JSON.stringify(change_mode));
    //
    // let low_pulse = {
    //     channel: 0,
    //     current: 10,
    //     pulse_width: 10
    // }
    // socket.emit("stimulator_low_pulse", JSON.stringify(low_pulse));
    //
    // let mid_pulse = {
    //     channel: 0,
    //     current: 10,
    //     pulse_width: 10,
    //     period: 10,
    //     ms: 10
    // }
    // socket.emit("stimulator_mid_pulse_timed", JSON.stringify(mid_pulse));
    //
    // let switch_button = {
    //     button: "3,0",
    //     status: "pos"
    // }
    // socket.emit("switch", JSON.stringify(switch_button));

    // socket.disconnect();
});