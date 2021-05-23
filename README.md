# FES Support Code
## Hamish Bultitude
Included in this repository are all the files required for running the FES interface & backend for my SCIE3221 research thesis.

Here is how things work.
### Frontend
1. `cd frontend/src/`
2. `yarn install`
3. `yarn start`
4. open a new terminal
5. `cd frontend/src/ && node api.js`

### Backend
1. `cd backend/fes`
2. `python3 controller.py`

Make sure to specify what port the stimulator is connected on (check COM ports on Windows, or tty ports on linux/mac)
Further, install all necessary drives for the U2715a Switch Matrix.
