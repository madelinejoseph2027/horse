## Horse

Horse is a set of pybullet/pyrosim-based functions that produces a 3-dimensional robot comprised of randomly-sized rectangular pieces. Links are sensorized at random, while joints are motorized at random. A sensing-enabled link is distinguished from a non-sensing link by its color; the former is green, while the latter is blue. 

In brief, the horse algorithm works by pre-defining a thoracic length of 2 to 6 links. The links' dimensions can range in size from 0.1 to 3.0 units. The algorithm then adds "limbs" into x, y, and z directions. These segments are random in size, but are **bilaterally symmetric** (to mimic most animals), such that if a "limb" exists on one side of the horse, its mirror image exists on the other. Sensors, motors, and synaptic weights are assigned at random based on a coin toss (boolean variable randomly set to 0 or 1).

The following diagram shows an example "horse." Green links are sensorized, while blue are unsensorized. Green joints are motorized, while blue are unmotorized (this wouldn't actually be reflected in the output of horseSim.py – joints are not colored). Synapses are active if they connect to a sensorized link with a motorized joint. A synapse that connect a unsensorized and/or unmotorized joints are inactive. The mirror plane, which defines the bilateral symmetry of the "horse," runs perpendicular to the viewer's line of vision. 

![Body-Brain Diagram](https://user-images.githubusercontent.com/122245493/220243994-f18b9ff8-2993-41eb-a7e1-76335d226b88.jpg)



## Usage

Please make sure all of the attending files from this repository are present in your working directory (e.g., by cloning this repo to your local system).

Run the following in the command line:

```bash
python3 horseSim.py
```


## Output

The the simulation is run 5 times to produce 5 unique, random "horses." All of these will be displayed in the pybullet GUI. This value can be changed in ```horseSim.py``` (in the for loop) if more iterations of simulations are desired. 


## Citations
Bongard, J. [u/DrJosh]. “Education in Evolutionary Robotics” Reddit, 6 Feb. 2023, https://www.reddit.com/r/ludobots/.

Kriegman, S. Artificial Life, Northwestern University, Evanston, Illinois, Winter 2023.
