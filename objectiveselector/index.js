// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const selectedIndexTopic = "/objectiveTracker/currentIndex";
const selectedSideTopic = "/objectiveTracker/currentSide";
const numElements = 8;

const timeout = 250;

let lastIndex = 0;

function displayTarget(index) {
    Array.from(document.getElementsByClassName("active")).forEach((element) => {
      element.classList.remove("active");
    });
  if (index !== null) {
    document.getElementsByTagName("td")[index].classList.add("active");
  }
}
function updateReefSide(side) {
  document.getElementsByTagName("hexagon")[side - 1].classList.add("highlighted");
}

let client = new NT4_Client(
  "10.31.81.2",
  //"127.0.0.1",
  "NodeSelector",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === selectedIndexTopic) {
      document.body.style.backgroundColor = "black";
      displayTarget(value);
      lastIndex = value;
    }
    if (topic.name === selectedSideTopic) {
      updateReefSide(value);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    displayTarget(null);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe([selectedIndexTopic], false, false, 0.02);
  client.subscribe([selectedSideTopic], false, false, 0.02);
  client.connect();

  addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });
});