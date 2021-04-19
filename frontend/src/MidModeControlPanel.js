import React, {useEffect} from "react";
import { makeStyles } from "@material-ui/core/styles";
import Grid from "@material-ui/core/Grid";
import Typography from "@material-ui/core/Typography";
import Slider from "@material-ui/core/Slider";
import PinDropIcon from "@material-ui/icons/PinDrop";
import Checkbox from "@material-ui/core/Checkbox";
import Switch from "@material-ui/core/Switch";
import FormGroup from "@material-ui/core/FormGroup";
import FormControlLabel from "@material-ui/core/FormControlLabel";
import FormControl from "@material-ui/core/FormControl";
import green from "@material-ui/core/colors/green";
import VariableReadOut from "./VariableReadOut";
import WavesIcon from "@material-ui/icons/Waves";
import OfflineBoltIcon from "@material-ui/icons/OfflineBolt";
import Button from "@material-ui/core/Button";
import TimelineIcon from "@material-ui/icons/Timeline";
import AccessTimeIcon from "@material-ui/icons/AccessTime";
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

export default function ControlPanel(props) {
  const classes = useStyles();
  let socket = null;
  const currentRef = React.useRef(null);
  const pulseWidthRef = React.useRef(null);
  const periodRef = React.useRef(null);
  const totalRuntimeRef = React.useRef(null);

  useEffect(() => {
    socket = io(ENDPOINT, {
      withCredentials: true,
      extraHeaders: {
        "my-custom-header": "abcd"
      }
    });
    let default_mode = {
      channel: 0,
      current: 0,
      pulse_width: 0,
      period: 0,
    }
    socket.emit("change", JSON.stringify(default_mode));
  }, []);




  const [pulseMode, setPulseMode] = React.useState({ pulseChecked: false });
  const handlePulseModeChange = (event) => {
    setPulseMode({ ...pulseMode, [event.target.name]: event.target.checked });
  };

  const handleRunClick = (event) => {
    console.log(currentRef);
    // mid level
    let stimulator_mid_pulse_timed = {
      channel: 1,
      current: currentRef.current.state.value,
      pulse_width: pulseWidthRef.current.state.value,
      period: periodRef.current.state.value,
      ms: totalRuntimeRef.current.state.value
    }
    socket.emit("stimulator_mid_pulse_timed", JSON.stringify(stimulator_mid_pulse_timed));
  }


  return (
    <div className={classes.root}>
      <Grid container spacing={2} alignItems="center">
        <Grid item>{props.icon}</Grid>
        <Grid item xs>
          <Typography component="div">
            <Grid component="div" container alignItems="center" spacing={1}>
              <Grid item>Run</Grid>
              <Grid item>
                <Switch
                  checked={pulseMode.pulseChecked}
                  onChange={handlePulseModeChange}
                  name="pulseChecked"
                />
              </Grid>
              <Grid item>Start/Stop</Grid>
              <Grid item>
                <VariableReadOut
                  label={"Current (mA)"}
                  step={1}
                  initial={5}
                  min={1}
                  max={30}
                  icon={<OfflineBoltIcon />}
                  ref={currentRef}
                />
                <VariableReadOut
                  label={"Pulse Width (μs)"}
                  step={10}
                  initial={350}
                  min={200}
                  max={500}
                  icon={<WavesIcon />}
                  ref={pulseWidthRef}
                />
                <VariableReadOut
                  label={"Period (ms)"}
                  step={1}
                  initial={30}
                  min={20}
                  max={50}
                  icon={<TimelineIcon />}
                  ref={periodRef}
                />
                {!pulseMode.pulseChecked ? (
                  <Grid item>
                    <VariableReadOut
                      label={"Total Runtime (ms)"}
                      step={1000}
                      initial={1000}
                      min={1000}
                      max={20000}
                      icon={<AccessTimeIcon />}
                      ref={totalRuntimeRef}
                    />
                    <Button variant="contained" onClick={handleRunClick}>RUN (Total Runtime)</Button>
                  </Grid>
                ) : (
                  <Grid item>
                    <Button variant="contained">START</Button>
                    <Button variant="contained">STOP</Button>
                  </Grid>
                )}
              </Grid>
            </Grid>
          </Typography>
        </Grid>
        <Grid item></Grid>
      </Grid>
    </div>
  );
}
