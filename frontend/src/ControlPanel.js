import React, {useContext, useEffect} from "react";
import { makeStyles } from "@material-ui/core/styles";
import Grid from "@material-ui/core/Grid";
import Typography from "@material-ui/core/Typography";
import Switch from "@material-ui/core/Switch";
import LowModeControlPanel from "./LowModeControlPanel";
import MidModeControlPanel from "./MidModeControlPanel";

import {SocketContext} from "./App";
import io from "socket.io-client";
const ENDPOINT = "http://127.0.0.1:8000";

const useStyles = makeStyles({
  root: {
    width: 300
  },
  input: {
    width: 42
  }
});

let socket = null;

export default function ControlPanel(props) {
  const classes = useStyles();

  const context = useContext(SocketContext);

  const [level, setLevel] = React.useState({ levelChecked: false });

  useEffect(() => {
    socket = io(ENDPOINT, {
      withCredentials: true,
      extraHeaders: {
        "my-custom-header": "abcd"
      }
    });
    let default_mode = {
      mode: 0
    }
    socket.emit(default_mode);
  }, []);

  const handleLevelChange = (event) => {
    setLevel({ ...level, [event.target.name]: event.target.checked });
    let change_mode = null;
    if (event.target.checked === false) {
      // low level
      change_mode = {
        mode: 0
      }
    } else {
      // mid level
      change_mode = {
        mode: 1
      }
    }
    socket.emit("stimulator_change_mode", JSON.stringify(change_mode));
  };

  return (
    <div className={classes.root}>
      <Typography id="control-panel" gutterBottom>
        Stimulator Level Mode
      </Typography>
      <Grid container spacing={2} alignItems="center">
        <Grid item>{props.icon}</Grid>
        <Grid item xs>
          <Typography component="div" alignCentre>
            <Grid component="div" container alignItems="center" spacing={1}>
              <Grid item>Low</Grid>
              <Grid item>
                <Switch
                  checked={level.levelChecked}
                  onChange={handleLevelChange}
                  name="levelChecked"
                />
              </Grid>
              <Grid item>Mid</Grid>
            </Grid>
          </Typography>
        </Grid>
        <Grid item></Grid>
      </Grid>
      {!level.levelChecked ? (
        <LowModeControlPanel> </LowModeControlPanel>
      ) : (
        <MidModeControlPanel> </MidModeControlPanel>
      )}
    </div>
  );
}
