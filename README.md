# Control in Human Robot Interaction
Calligraphy Project | Group 7

## Table of contents
1. [About](#about)
2. [Method](#method)
3. [Setup](#setup)
4. [Usage](#usage)
5. [Results](#results)
6. [Conclusion](#conclusion)
7. [Recommendations](#recommendations)
8. [Acknowledgements](#acknowledgements) 

---

## About
Python implementation of haptic feedback for calligraphy learning by motor control training. This project uses a [Haply](https://www.haply.co/) device.

This project is a research project answering the following hypothesis:

> *It is possible to learn previously unseen calligraphic characters via force feedback*

## Features to extend

> *The thickness of the gradient force field of the lines can be thinner at each iteration to see what happens for (learning) performance*
> *How does the (leanrning) performance change when a disturbance is added to the simulation*
---

## Method
Step-by-step overview of our proposed method to answer the hypothesis
- [ ] Build Haply interface with impedance control in Python
- [ ] Add calligraphy shapes
- [ ] Test hypothesis on test subjects
- [ ] Answer hypothesis

---

## Setup
1. Clone this repository to your computer.
2. Install all dependencies
```
pip install -r requirements.txt
```
3. Connect the Haply to the computer. Make sure the Haply is powered on.
4. Launch the Haply interface
```Python
python3 haply_calligraphy.py
```
5. Learn how to make beautiful calligraphy

## Usage
The Haply interface can be controlled with the keyboard.

- Press 'e' to start the excersise
- Press 'q' to exit the excersise
- Press 'r' to reset the current excersis
- Press 'n' to go to the next character
- Press 'd' to toggle character display

---

## Results
Insert figures/tables

## Conclusion
Hypothesis accepted or rejected?

## Recommendations
How to improve research method?
Further look into...

---

## Acknowledgements
We would like to thank dr. ir. Wiertlewski and dr. ir. Peternel for their support and feedback throughout our research.
