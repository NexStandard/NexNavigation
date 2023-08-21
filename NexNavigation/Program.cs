// See https://aka.ms/new-console-template for more information
// Create a sample navigation map
using NexNavigation;
using Stride.Core.Mathematics;

MapBuilder mapBuilder = new MapBuilder();
List<Vector3> vertices = new List<Vector3>
        {
            new Vector3(0, 0, 0),
            new Vector3(1, 0, 0),
            new Vector3(0, 1, 0),
            new Vector3(1, 1, 0)
        };
List<int> indices = new List<int> { 0, 1, 2, 1, 3, 2 };
var nav = new NavigationAgentSettings()
{
    Height = 1,
    Radius = 0.24f,
    
};
var navigationMap = mapBuilder.BuildNavigationMap(vertices, indices);

// Create an AStar instance
AStar aStar = new AStar();

// Define start and goal positions
Vector3 start = new Vector3(0, 0, 0);
Vector3 goal = new Vector3(1, 1, 0);

// Find the path
List<Vector3> path = aStar.FindPath(navigationMap, start, goal,100);

// Print the path
if (path != null)
{
    Console.WriteLine("Path found:");
    foreach (Vector3 point in path)
    {
        Console.WriteLine($"({point.X}, {point.Y}, {point.Z})");
    }
}
else
{
    Console.WriteLine("Path not found.");
}