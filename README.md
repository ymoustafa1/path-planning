# Path Planning

A simple solution to the cone path planning assignment.

## Approach

Blue cones mark the left side and yellow cones mark the right side. The planner first expresses the cones relative to the car, ignores cones behind it, and sorts the remaining cones from nearest to farthest ahead.

It joins cones on each side with straight lines and takes the middle between the two sides. If only one side is visible, it assumes a 2 metre track width and shifts the path 1 metre inward. With no forward cones, it goes straight along the car's heading.

Three cones on one side form two connected boundary segments, so the same method follows a simple bend. The planner joins the car to the estimated centre, extends the last segment, and returns an 8 metre path in world coordinates with points no more than 0.5 metres apart.

I chose this approach because it uses basic geometry and works with the small number of cones in this task.

## Run

Use Python 3.9 or later. From the project folder:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m src.run --scenario 23
```

Scenarios 1–20 are the supplied examples. Scenarios 21–26 add three blue cones, three yellow cones, bends, both sides, and a car with a different position and heading.

In PyCharm, open this folder, select `.venv/bin/python` as the interpreter, and run the module `src.run` with `--scenario 23` as its parameters.

## Check

```sh
python -m unittest discover -s tests -v
```

## Assumptions and limits

The car is assumed to approach a track that progresses forward along its heading. Cone colours are correct and the car can reach the first estimated centre without crossing a boundary. Track width is assumed to be 2 metres when one side is missing, measured across the car's heading.

The path uses straight segments, so corners can be abrupt. It does not account for steering limits, car size, speed, or obstacles. Tight turns and cones at the same forward distance are ambiguous. Cones behind the car are ignored, and extending the last segment is only an estimate of the unseen track. Some supplied scenarios have a heading that does not match the apparent track; this simple approach cannot handle every layout.
