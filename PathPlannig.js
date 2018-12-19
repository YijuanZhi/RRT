//Tree class that used to grow on the map looking for the goal ============================================
class Tree{
  constructor(node, parent) {
    this.node = node;         //The Point that represents the 2D location of the tree's current node.
    this.parent = parent;     //Reference to the instance of the Tree class that is the parent of the current node.
    this.children = [];       //An array of instances of Trees, which correspond to the children of the tree’s current node.
  }
  nearest(p) {                //Finding the nearest tree object(this or this.children) that it's node is nearest to the point p
    //CHECK
    let d = distance(this.node, p);
    let best = this;
    for (let i = 0; i < this.children.length; ++i) {
      let other = this.children[i].nearest(p);    //where recursion happens.
      if (distance(other.node, p) < d) {
        best = other;
        d = distance(other.node, p);
      }
    }
    return best;
  }
  extend(p, maxExtension) {   //Tree extension, up to maxExtension. If p is closer than maxExtension, then return p instead.
    //CHECK
    let q = this.node;
    let d = distance(p,q);
    let r = p;
    if (d < maxExtension) {
      return r;
    } else {
      let f = maxExtension / d;
      r = new geometry.Point(q.x + f * (p.x - q.x), q.y + f * (p.y - q.y));
    }
    return r;
  }
  add(p) {   //add a point to the tree as a children
    //CHECK
    let qNew = new Tree(p, this);
    this.children.push(qNew);
    return qNew;
  }
}

//Helper functions part===============================================================================

//Takes two points, 
//and returns the distance between the two points given by the formula √(p1.x − p2.x)^2 + (p1.y − p2.y)^2
function distance(p1, p2){
  //CHECK
  let a = p1.x - p2.x;
  let b = p1.y - p2.y;
  let d = Math.sqrt(a*a+b*b);
  return d;
}


//takes the size of the map, the goal point, and the goal bias, 
//and returns a randomly sampled 2D point within the map. A certain fraction of the time (specified by goalBias), 
//this function will return the goal point itself.
function samplePoint(mapSize, goal, goalBias){    // goalbias is the parameter controling the probability of 
  //CHECK                                         // the sameple point being the goal point
  const r = Math.random();
  if (r < goalBias) {
    return goal;
  }
  return new geometry.Point(mapSize * Math.random(), mapSize * Math.random());
}


//takes in a map as an array of line segments, 
//and returns true if the line segment joining p1 and p2 
//intersects with any of the line segments in the map, and false otherwise.
function collides(maplines, p1, p2){
  //CHECK
  let tl = new geometry.Line(p1 ,p2);      //target line
  let reducer = function(status, mapline){
    let cur = geometry.intersects(mapline,tl);
    return (cur || status);
  };
  return maplines.reduce(reducer, false);
}

//Path part =============================================================================================

const emptyP = null;        //empty parent

//takes in a Tree leaf that is closest to the goal, 
//and returns the full path found, as an array of points. 
//The first point in the array must be the goal, and the last point the start point.  
//rerturn   goal --> start
function getPath(leaf, goal){  //?????
  //TODO

  let cur = leaf.nearest(goal);     //start from the goal point
  let inversepath = [];
  inversepath.push(goal);
  if(cur.node.x === goal.x && cur.node.y === goal.y){
    cur = cur.parent;
  }
  while(true){
    inversepath.push(cur.node);
    cur = cur.parent;
    if(cur === null){             //when the empty parent is reached
      break;
    }
  }
  return inversepath;
}



//To put everything together:
//The plan function should return the collision free path as an array of 
//points with the first point in the list being the start and the last being the goal.
//return start --> goal
function plan(start, goal, map, options){
  //CHECK
  let f = options.callback;
  let tree = new Tree(start, emptyP);                                //create a new tree head with empty parent
  let cur =tree;                                                     //the node which will finally become the goal
  let path = [];                                                     //the path we want to return
  let found = false;
  let attempt = 0;
  while(true){
    let p = samplePoint(options.mapSize, goal, options.goalBias);    //create a new sample point
    let near = tree.nearest(p);                                      //get the tree node that is closest to the sample point 
    let newn = near.extend(p, options.maxExtension);                 //the point we intend to expand to(before checking)
    if(!collides(map, near.node, newn)){                             //check if the near node and newn node collides with the map
      let cur = near.add(newn);                                      //add the newn to the tree and assign it to cur
      attempt = attempt + 1;
      if(newn.x === goal.x && newn.y === goal.y){                    //when the goal is reached
        found = true;
        f(p, near, newn, tree);
        break;
      }
      else if(attempt === options.maxSamples){                         //when max attempt reached
        f(p, near, newn, tree);
        break;
      }
      f(p, near, newn, tree);                                        //callback function
    }
  }
  //when the situation when the goal is not found(for instance, maxSamples is reached),
  //we need to return a path that starts with the start point and ends with the point that ios nearest to the goal.
  if(!found){
    let inversepath = getPath(tree, goal);
    for(let i = inversepath.length -1 ; i > 0; --i){
      path.push(inversepath[i]);
    }
    return path;
  }
  //creating the final path with head and goal using getPath function
  let inversepath = getPath(tree, goal);
  for(let i = inversepath.length -1 ; i > -1; --i){
    path.push(inversepath[i]);
  }
  return path;
}

//option object description
/*
let options = {
  mapSize: 400,                
  maxExtension: 10,
  goalBias: 0.05,
  maxSamples: 10000,            //The maximum number of samples that the RRT planner should generate, to find a plan.
  callback: emptyCallback       
};
let emptyCallback = function(p, q, r, t) {
  //A callback function that must be called at every iteration of the RRT algorithm, with the following parameters:
  //p, the current sampled point
  //q, the closest leaf in the tree to p
  //r, the extension point from q to r
  //t, the current tree grown so far.
};
*/






// Test part......

/*
//The canvas and map that will be used later in tests -----------------------------------------------------------
let c0 = lib220.newCanvas(400,400);
let map = [
  new geometry.Line(new geometry.Point(0, 0), new geometry.Point(0, 400)),
  new geometry.Line(new geometry.Point(400, 400), new geometry.Point(0, 400)),
  new geometry.Line(new geometry.Point(400, 400), new geometry.Point(400, 0)),
  new geometry.Line(new geometry.Point(400, 0), new geometry.Point(0, 0)),
  new geometry.Line(new geometry.Point(0, 200), new geometry.Point(300, 200)),
  new geometry.Line(new geometry.Point(0, 100), new geometry.Point(200, 100)),
  new geometry.Line(new geometry.Point(300, 100), new geometry.Point(400, 100)),
  new geometry.Line(new geometry.Point(100, 200), new geometry.Point(100, 300)),
  new geometry.Line(new geometry.Point(200, 300), new geometry.Point(200, 400)),
  new geometry.Line(new geometry.Point(300, 300), new geometry.Point(400, 300)),
];
let len = map.length;
//drawing the map
for(let i = 0; i < len; ++i){
  c0.drawLine(map[i].p1.x, map[i].p1.y, map[i].p2.x, map[i].p2.y, [0,0,0]);
}
*/


// helper functions test -------------------------------------------------------------------------

//test of samplePoint function
test('Must return goal', function() {
  let goal = new geometry.Point(0, 0);
  let p = samplePoint(400, goal, 1); 
  assert(p.x === goal.x && p.y === goal.y);
});

test('intersection test', function(){
  let l1 = new geometry.Line(new geometry.Point(0, 0), new geometry.Point(0, 10));
  let l2 = new geometry.Line(new geometry.Point(0, 0), new geometry.Point(10, 0));
  assert(geometry.intersects(l1, l2) === true)
});

/*
//visual test for sampling
//CHECK
let goal = new geometry.Point(100, 100);
let goalBias = 0.8;                                 // parameter to change
for (let i = 0; i < 10000; ++i) {
  let p = samplePoint(400, goal, goalBias);
  if (p.x === goal.x && p.y === goal.y) {
    c0.drawFilledCircle(p.x, p.y, 2, [1, 0, 0]);     //draw a point with a red circle
  } else {
    c0.drawFilledCircle(p.x, p.y, 2, [0, 0, 1]);     //blue circle
  }
  //lib220.sleep(10);
}
*/

/*
//visual test for collision
//CHECK
let lines = [];
for (let i = 0; i < 20; ++i) {
  let newp1 = new geometry.Point(400 * Math.random(), 400 * Math.random());
  let newp2 = new geometry.Point(400 * Math.random(), 400 * Math.random());
  let newl = new geometry.Line(newp1, newp2);
  lines.push(newl);
}
lines.forEach(function(l) {
  let color = [0, 0, 1];
  if (collides(map, l.p1, l.p2)) {
    color = [1, 0, 0];
  }
  c0.drawLine(l.p1.x,l.p1.y, l.p2.x,l.p2.y, color);
});
*/


//tree part and path part test -------------------------------------------------------------------

//helper functions for visualize callback function 
function drawMap(c, map){
  for(let i = 0; i < map.length; ++i){
    c.drawLine(map[i].p1.x, map[i].p1.y, map[i].p2.x, map[i].p2.y, [0,0,0]);
  }
}

function drawPoint(c, p, color){
  c.drawFilledCircle(p.x, p.y, 2, color);
}

function drawTree(c, t){
  let cl = t.children.length;
  for(let i =0; i<cl; ++i){
    let child = t.children[i];
    c.drawLine(t.node.x, t.node.y, child.node.x, child.node.y, [0,0,1]);
    drawTree(c, child);    //recursions for the children
  }
}

function drawLineM(c, p1, p2, color){
  c.drawLine(p1.x, p1.y, p2.x, p2.y, color);
}


//visualiztion of the tree. An example of callback function
function visualize(p, q, r, t) {
  c0.clear();
  drawMap(c0, map);
  drawPoint(c0, start, [1, 0, 0]);
  drawPoint(c0, goal, [0, 0.75, 0]);
  drawPoint(c0, p, [0.5, 0.5, 0]);
  drawPoint(c0, q.node, [0, 1, 0.5]);
  drawPoint(c0, r, [0.25, 0.5, 0]);
  drawLineM(c0, q.node, r, [1, 0, 0]);
  drawLineM(c0, p, r, [1, 0.5, 0.8]);
  drawTree(c0, t);
  //lib220.sleep(1);  //drawing animation
}

let options0 = {
  mapSize: 400,                
  maxExtension: 10,
  goalBias: 0.05,
  maxSamples: 5000,
  callback: visualize
};



/*
//verify the path has no collision with map
function verifyPath(path, map) {
  if (path.length < 1) {
    assert(true);
    return; 
  }
  let init = {
    collide: false,
    prev: path[0]
  }
  let reducer = function(check, p) {
    check.collide = check.collide || collides(map, p, check.prev);
    check.prev = p;
    return check;
  };
  let result = path.reduce(reducer, init);
  assert(!result.collide);
  //return result.collide;
}
*/
/*
//n times verifying path
for (let i = 0; i < 10; ++i) {
  test('Verify path' + i.toString(), function() {
    let start = new geometry.Point(50, 50);
    let goal = new geometry.Point(50,250);
    let options = options0;
    let path = plan(start, goal, map, options);
    verifyPath(path, map);
    if (path.length >= 2) {
      assert(path[0].x === goal.x && path[0].y === goal.y);
      let lastPoint = path[path.length - 1];
      assert(lastPoint.x === start.x && lastPoint.y === start.y);
    }
  });
  console.log(i);
}
*/

/*
let start = new geometry.Point(50, 50);
let goal = new geometry.Point(50,250);
let options = options0;
for (let i = 0; i < 10; ++i) {
  test('Verify path' + i.toString(), function() {
    let path = plan(start, goal, map, options);
    verifyPath(path, map);
    
    if (path.length >= 2) {
      assert(path[0].x === start.x && path[0].y === start.y);
      let lastPoint = path[path.length - 1];
      assert(lastPoint.x === goal.x && lastPoint.y === goal.y);
    }
    
  });
  console.log(i);
}
*/



/*
//just playing around
let start = new geometry.Point(50, 50);
let goal = new geometry.Point(50,250);
let options = options0;
let path = plan(start, goal, map, options);
console.log(path[0]);
let plength = path.length;
console.log(path[plength-1]);
//console.log(path);
//console.log(verifyPath(path, map));
*/