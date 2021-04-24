import React, {useEffect} from "react";
import { makeStyles, MuiThemeProvider } from "@material-ui/core/styles";
import Grid from "@material-ui/core/Grid";
import Button from "@material-ui/core/Button";
import { createMuiTheme } from "@material-ui/core/styles";
import red from "@material-ui/core/colors/red";
import green from "@material-ui/core/colors/green";
import grey from "@material-ui/core/colors/grey";
import io from "socket.io-client";
const ENDPOINT = "http://127.0.0.1:8000";
// const api = require("./api");

const useStyles = makeStyles((theme) => ({
  root: {
    flexGrow: 1
  },
  paper: {
    padding: theme.spacing(1),
    textAlign: "center",
    color: theme.palette.text.secondary
  }
}));

export default function NestedGrid(props) {
  const classes = useStyles();
  var columnNumb = [...Array(props.columnNumb).keys()];
  var rowNumb = [...Array(props.rowNumb).keys()];
  let socket = null;

  useEffect(() => {
    socket = io(ENDPOINT, {
      withCredentials: true,
      extraHeaders: {
        "my-custom-header": "abcd"
      }
    });
  }, []);

  const instantiateMatrix = () => {
    let dict = {};
    for (let y = 0; y < props.rowNumb; y++) {
      for (let x = 0; x < props.columnNumb; x++) {
        let key = `${x}:${y}`;
        dict[key] = 0;
      }
    }
    return dict;
  };

  const [arrayDict, setArrayDict] = React.useState(instantiateMatrix());

  const updateMatrix = (x, y, e) => {
    console.log(e.target.innerText);
    if (socket == null) {
      socket = io(ENDPOINT, {
        withCredentials: true,
        extraHeaders: {
          "my-custom-header": "abcd"
        }
      });
    }
    const key = `${y}:${x}`;
    let newState;
    setArrayDict((prevState) => {
      if (arrayDict[key] === 0) {
        newState = 1
      } else {
        newState = 0
      }

      return { ...prevState, [key]: newState };
    });
    console.log(`${y}:${x} -> ${newState}`);
    const data = {
      // button: key,
      button: e.target.innerText,
      state: newState
    }
    socket.emit("switch_button", JSON.stringify(data));
  };

  const buttonTheme = createMuiTheme({
    palette: { primary: grey, secondary: green, }
  });

  const naming = (key) => {
    let label = "empty";
    switch(key) {
      case "0:0":
        label = "1";
        break;
      case "1:0":
        label = "2";
        break;
      case "0:1":
        label = "3";
        break;
      case "1:1":
        label = "4";
        break;
      case "0:2":
        label = "5";
        break;
      case "1:2":
        label = "6";
        break;
      case "0:3":
        label = "7";
        break;
      case "1:3":
        label = "8";
        break;
      case "0:4":
        label = "x";
        break;
      case "1:4":
        label = "y";
        break;
      default:
        label = key;
        break;
    }
    return label;
  }

  return (
    <div className={classes.root}>
      <Grid container spacing={1}>
        {columnNumb.map((y) => (
          <Grid container item xs={12} spacing={1}>
            {rowNumb.map((x) => (
              <Grid item xs={1.5}>
                <MuiThemeProvider theme={buttonTheme}>
                  <Button
                    variant="contained"
                    onClick={(e) => updateMatrix(x, y, e)}
                    color={arrayDict[`${y}:${x}`] ? "secondary" : "primary"}
                  >
                    {naming(`${x}:${y}`)}
                  </Button>
                </MuiThemeProvider>
              </Grid>
            ))}
          </Grid>
        ))}
      </Grid>
    </div>
  );
}
