using Stride.Core.Mathematics;

public class AStar
{
    public List<Vector3> FindPath(Dictionary<Vector3, List<int>> navigationMap, Vector3 start, Vector3 goal, float maxSlope)
    {
        PriorityQueue<Node> openSet = new PriorityQueue<Node>();
        HashSet<Vector3> closedSet = new HashSet<Vector3>();
        Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
        Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float>();

        openSet.Enqueue(new Node(start, Heuristic(start, goal), 0, 0));

        gScore[start] = 0;

        while (openSet.Count > 0)
        {
            Node current = openSet.Dequeue();

            if (current.Position == goal)
                return ReconstructPath(cameFrom, current.Position);

            closedSet.Add(current.Position);

            foreach (int neighborIndex in navigationMap[current.Position])
            {
                Vector3 neighbor = navigationMap.Keys.ToArray()[neighborIndex];
                float neighborSlope = CalculateSlope(current.Position, neighbor);

                if (Math.Abs(neighborSlope) > maxSlope)
                    continue;

                float tentativeGScore = gScore[current.Position] + Distance(current.Position, neighbor);

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current.Position;
                    gScore[neighbor] = tentativeGScore;

                    float fScore = tentativeGScore + Heuristic(neighbor, goal);
                    openSet.Enqueue(new Node(neighbor, fScore, tentativeGScore, neighborSlope));
                }
            }
        }

        return null; // Path not found
    }

    private List<Vector3> ReconstructPath(Dictionary<Vector3, Vector3> cameFrom, Vector3 current)
    {
        List<Vector3> path = new List<Vector3>();
        while (cameFrom.ContainsKey(current))
        {
            path.Add(current);
            current = cameFrom[current];
        }
        path.Reverse();
        return path;
    }

    private float Distance(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b);
    }

    private float Heuristic(Vector3 a, Vector3 b)
    {
        return Distance(a, b);
    }

    private float CalculateSlope(Vector3 pointA, Vector3 pointB)
    {
        float heightDifference = Math.Abs(pointA.Y - pointB.Y);
        float horizontalDistance = Vector2.Distance(new Vector2(pointA.X, pointA.Z), new Vector2(pointB.X, pointB.Z));
        float slopeAngleRad = (float)Math.Atan(heightDifference / horizontalDistance);
        float slopeAngleDeg = (float)(slopeAngleRad * (180f / Math.PI));
        return slopeAngleDeg;
    }

    private class Node : IComparable<Node>
    {
        public Vector3 Position { get; }
        public float FScore { get; }
        public float GScore { get; }
        public float Slope { get; }

        public Node(Vector3 position, float fScore, float gScore, float slope)
        {
            Position = position;
            FScore = fScore;
            GScore = gScore;
            Slope = slope;
        }

        public int CompareTo(Node other)
        {
            return FScore.CompareTo(other.FScore);
        }
    }

    private class PriorityQueue<T> where T : IComparable<T>
    {
        private List<T> items = new List<T>();

        public int Count => items.Count;

        public void Enqueue(T item)
        {
            items.Add(item);
            items.Sort();
        }

        public T Dequeue()
        {
            T item = items[0];
            items.RemoveAt(0);
            return item;
        }
    }
}