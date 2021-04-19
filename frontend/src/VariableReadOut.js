import React from "react";
import { makeStyles } from "@material-ui/core/styles";
import Grid from "@material-ui/core/Grid";
import Typography from "@material-ui/core/Typography";
import Slider from "@material-ui/core/Slider";
import Input from "@material-ui/core/Input";
import {withStyles} from "@material-ui/core";

// const useStyles = makeStyles({
//   root: {
//     width: 300
//   },
//   input: {
//     width: 42
//   }
// });

const styles = theme => ({
    root: {
        width: 300
    },
    input: {
        width: 42
    }
});

class VariableReadOut extends React.Component  {

    constructor(props){
        super(props);
        this.state = {
            value: props.initial
        }
    }

    componentDidMount() {

    }


  render() {
      // const classes = useStyles();
      // const [value, setValue] = React.useState(this.props.initial);

      const handleSliderChange = (event, newValue) => {
          // setValue(newValue);
          this.setState({
              value: newValue,
          })
      };

      const handleInputChange = (event) => {
          this.setState({
              value: event.target.value === "" ? "" : Number(event.target.value)
          })
          // setValue(event.target.value === "" ? "" : Number(event.target.value));
      };

      const handleBlur = () => {
          if (this.state.value < 0) {
              // setValue(0);
              this.setState({
                  value: 0,
              })
          } else if (this.state.value > 100) {
              // setValue(100);
              this.setState({
                  value: 100,
              })
          }
      };
      
      const { classes }= this.props;
      return (
          <div className={classes.root}>
              <Typography id="input-slider" gutterBottom>
                  {this.props.label}
              </Typography>
              <Grid container spacing={2} alignItems="center">
                  <Grid item>{this.props.icon}</Grid>
                  <Grid item xs>
                      <Slider
                          value={typeof this.state.value === "number" ? this.state.value : 0}
                          onChange={handleSliderChange}
                          aria-labelledby="input-slider"
                          marks
                          step={this.props.step}
                          min={this.props.min}
                          max={this.props.max}
                          valueLabelDisplay="auto"
                      />
                  </Grid>
                  <Grid item>
                      <Input
                          className={classes.input}
                          value={this.state.value}
                          margin="dense"
                          onChange={handleInputChange}
                          onBlur={handleBlur}
                          inputProps={{
                              step: this.props.step,
                              min: this.props.min,
                              max: this.props.max,
                              type: "number",
                              "aria-labelledby": "input-slider"
                          }}
                      />
                  </Grid>
              </Grid>
          </div>
      );
  }

}

export default withStyles(styles, { withTheme: true })(VariableReadOut);
