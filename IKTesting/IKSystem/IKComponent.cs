using Stride.Core;
using Stride.Engine;
using Stride.Engine.Design;
using QuikGraph;
using QuikGraph.Algorithms;
using QuikGraph.Utils;
using QuikGraph.Predicates;
using Stride.Rendering;
using System.Linq;
using System.Collections.Generic;
using QuikGraph.Algorithms.ShortestPath;
using Stride.Core.Mathematics;

namespace IKTesting
{
    [DataContract("IKComponent")]
    [DefaultEntityComponentProcessor(typeof(IKProcessor), ExecutionMode = ExecutionMode.Runtime | ExecutionMode.Thumbnail | ExecutionMode.Preview)]
    [Display("Inverse Kinematics", Expand = ExpandRule.Once)]
    [ComponentOrder(2005)]
    [ComponentCategory("Animation")]
    public class IKComponent : EntityComponent
    {
        private AdjacencyGraph<NodeData,Edge<NodeData>> _graph;
        private IVertexAndEdgeListGraph<NodeData,Edge<NodeData>> Graph 
        {
            get {return _graph;}
        }
        private TryFunc<NodeData, IEnumerable<Edge<NodeData>>> ShortestPath;
        private List<NodeData> Leaves;

        [DataMember("Number of iteration")]
        public uint NbIteration;
        public Dictionary<string,Entity> BoneToTarget = new();
        
        public void BuildGraph()
        {
            _graph = new();
            var sk = Entity.Get<ModelComponent>().Skeleton;
            var nodes = sk.Nodes.Select((x,i) => new NodeData{Index = i, Name = x.Name, Parent = x.ParentIndex, Node = sk.NodeTransformations[i]}).ToList();
            nodes.ForEach(
                x =>
                {
                    var parent = _graph.Vertices.FirstOrDefault(e => e.Index == x.Parent);
                    // skGraph.AddVertex(x);
                    if(parent != null)
                    {
                        _graph.AddVertex(x);
                        parent.Distance = Vector3.Distance(parent.Node.Transform.Position,x.Node.Transform.Position); //Vector3.Distance(parent.Node.WorldMatrix.TranslationVector, x.Node.WorldMatrix.TranslationVector);
                        _graph.AddEdge(new Edge<NodeData>(parent,x));
                    }
                    else
                    {
                        _graph.AddVertex(x);
                    }
                }
            );
            Leaves = _graph.Vertices.Where(x => !_graph.Edges.Any(e => e.Source.Index == x.Index)).Where(x => BoneToTarget.Keys.Contains(x.Name)).ToList();
            ShortestPath = Graph.ShortestPathsDijkstra(_ => 1, _graph.Vertices.First());
        }

        public void ComputeFabrik()
        {
            foreach(var (n,e) in BoneToTarget)
            {
                ShortestPath(_graph.Vertices.First(x => x.Name == n), out var p);
                var path = p.ToList();
                var tmp1 = new FabrikData[path.Count +1];
                var tmp2 = new FabrikData[path.Count +1];
                tmp1[0] = new FabrikData{Position = path[0].Source.Node.Transform.Position, Distance = path[0].Source.Distance};
                for (int i = 1; i < path.Count; i++)
                {
                    tmp1[i] = new FabrikData{Position = path[i].Target.Node.Transform.Position, Distance = path[i].Target.Distance};
                }
                tmp1[^0] = new FabrikData{Position = e.Transform.Position, Distance = Vector3.Distance(tmp1[^1].Position, e.Transform.Position)};
                for (int i = 0; i < NbIteration; i++)
                {
                    
                    for (int j = tmp1.Length-2; j > 0; j--)
                    {
                        
                    }
                    for (int j = 1; j < tmp1.Length; j++)
                    {
                        // tmp1[j] = 
                    }
                }
            }
            
        }
        public bool CheckValid()
        {
            var mc = Entity.Get<ModelComponent>();
            return mc != null && mc.Skeleton != null;
        }
        internal class NodeData
        {
            public int Index {get;set;}
            public string Name {get;set;}
            public int Parent {get;set;}
            public float Distance {get;set;}
            public ModelNodeTransformation Node {get;set;}
        }
        public class FabrikData
        {
            public Vector3 Position;
            public float Distance;
        }
    }
}