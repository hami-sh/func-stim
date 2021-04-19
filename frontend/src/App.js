import "./styles.css";
import React, {useState, useEffect} from "react";
import MatrixArrayReadOut from "./MatrixArrayReadOut";
import ControlPanel from "./ControlPanel";
// import socketIOClient from "socket.io-client";
import io from "socket.io-client";
const ENDPOINT = "http://127.0.0.1:8000";

let socket = null;
export const SocketContext = React.createContext({socket});

export default class App extends React.Component {

    constructor(props) {
        super(props);
    }

    componentDidMount() {
        socket = io(ENDPOINT, {
            withCredentials: true,
            extraHeaders: {
                "my-custom-header": "abcd"
            }
        });
    }

    render() {
        return (
            <div className="App">
                <SocketContext.Provider value={socket}>
                    <ControlPanel/>
                    <MatrixArrayReadOut rowNumb={8} columnNumb={1}/>
                </SocketContext.Provider>
            </div>
        );
    }
}
