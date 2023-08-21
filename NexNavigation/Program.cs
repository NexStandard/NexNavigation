// Create an instance of your AStar class
using Stride.Core.Mathematics;

AStar astar = new AStar();

// Define test values for navigation map and bounding boxes
Dictionary<Vector3, List<int>> navigationMap = new Dictionary<Vector3, List<int>>
        {
            { new Vector3(0, 0, 0), new List<int> { 1, 2 } }, // Assuming these are neighbor indices
            { new Vector3(1, 0, 0), new List<int> { 0, 2 } },
            { new Vector3(0, 0, 1), new List<int> { 0, 1 } }
        };

Dictionary<Vector3, BoundingBox> boundingBoxes = new Dictionary<Vector3, BoundingBox>
        {
            { new Vector3(0, 0, 0), new BoundingBox(new Vector3(-0.5f), new Vector3(0.5f)) },
            { new Vector3(1, 0, 0), new BoundingBox(new Vector3(0.5f, 0, -0.5f), new Vector3(1.5f, 1, 0.5f)) },
            { new Vector3(0, 0, 1), new BoundingBox(new Vector3(-0.5f, 0, 0.5f), new Vector3(0.5f, 1, 1.5f)) }
        };

Vector3 start = new Vector3(0, 0, 0);
Vector3 goal = new Vector3(0, 0, 0.75f);
float maxSlope = 30.0f; // Example maximum slope in degrees

// Call your FindPath method and process the result
List<Vector3> path = astar.FindPath(navigationMap, boundingBoxes, start, goal, maxSlope);

if (path != null)
{
    Console.WriteLine("Path found:");
    foreach (var point in path)
    {
        Console.WriteLine(point);
    }
}
else
{
    Console.WriteLine("Path not found.");
}