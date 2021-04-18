import React from "react";
import { makeStyles } from "@material-ui/core/styles";
import Grid from "@material-ui/core/Grid";
import Typography from "@material-ui/core/Typography";
import Slider from "@material-ui/core/Slider";
import PinDropIcon from "@material-ui/icons/PinDrop";
import Checkbox from "@material-ui/core/Checkbox";
import Switch from "@material-ui/core/Switch";

const useStyles = makeStyles({
  root: {
    width: 300
  },
  input: {
    width: 42
  }
});

export default function WaveformReadOut(props) {
  const classes = useStyles();

  const [value, setValue] = React.useState(props.initial);
  const handleSliderChange = (event, newValue) => {
    setValue(newValue);
  };

  const [state, setState] = React.useState({ checkedA: true });
  const handleChange = (event) => {
    setState({ ...state, [event.target.name]: event.target.checked });
  };

  return (
    <div className={classes.root}>
      <Typography id="input-slider" gutterBottom>
        {props.label}
      </Typography>
      <Grid container spacing={2} alignItems="center">
        <Grid item>
          <PinDropIcon />
        </Grid>
        <Grid item xs>
          <Slider
            // value={typeof value === "number" ? value : 0}
            defaultValue={props.initial}
            onChange={handleSliderChange}
            aria-labelledby="input-slider"
            marks
            step={props.step}
            min={props.min}
            max={props.max}
            track={false}
          />
        </Grid>
        <Grid item>
          <Switch
            checked={state.checkedA}
            onChange={handleChange}
            name="checkedA"
          />
        </Grid>
      </Grid>
    </div>
  );
}
