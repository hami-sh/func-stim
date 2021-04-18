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
        let key = `${y}:${x}`;
        dict[key] = 0;
      }
    }
    return dict;
  };

  const [arrayDict, setArrayDict] = React.useState(instantiateMatrix());

  const updateMatrix = (x, y) => {
    if (socket == null) {
      socket = io(ENDPOINT, {
        withCredentials: true,
        extraHeaders: {
          "my-custom-header": "abcd"
        }
      });
    }
    const key = `${x}:${y}`;
    // console.log(arrayDict);
    let newState;
    setArrayDict((prevState) => {
      // switch (arrayDict[key]) {
      //   case 0:
      //   case 1:
      //     newState= arrayDict[key]++;
      //     break;
      //   case 2:
      //     newState = 0;
      //     break;
      // }
      if (arrayDict[key] === 0) {
        newState = 1
      } else {
        newState = 0
      }

      return { ...prevState, [key]: newState };
    });
    console.log(`${x}:${y} -> ${newState}`);
    const data = {
      button: key,
      state: newState
    }
    socket.emit("switch_button", JSON.stringify(data));
  };

  const buttonTheme = createMuiTheme({
    palette: { primary: grey, secondary: green, tertiary: red}
  });

  return (
    <div className={classes.root}>
      <Grid container spacing={1}>
        {columnNumb.map((y) => (
          <Grid container item xs={12} spacing={3}>
            {rowNumb.map((x) => (
              <Grid item xs={2}>
                <MuiThemeProvider theme={buttonTheme}>
                  <Button
                    variant="contained"
                    onClick={() => updateMatrix(x, y)}
                    color={arrayDict[`${x}:${y}`] ? "secondary" : "primary"}
                  >
                    {x}:{y}
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
