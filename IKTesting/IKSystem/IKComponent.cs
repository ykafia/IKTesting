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
using Stride.Games;

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
                        parent.Distance = Vector3.Distance(parent.Node.WorldMatrix.TranslationVector,x.Node.WorldMatrix.TranslationVector); //Vector3.Distance(parent.Node.WorldMatrix.TranslationVector, x.Node.WorldMatrix.TranslationVector);
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

        public void ComputeFabrik(GameTime time)
        {
            foreach(var (n,e) in BoneToTarget)
            {
                ShortestPath(_graph.Vertices.First(x => x.Name == n), out var p);
                var d = p.Sum(x => x.Source.Distance);
                var sk = Entity.Get<ModelComponent>().Skeleton;
                var currNode = sk.NodeTransformations[24];
                var direction =  e.Transform.WorldMatrix.TranslationVector - currNode.WorldMatrix.TranslationVector;
                direction.Normalize();
                sk.NodeTransformations[24].Transform.Rotation = DirRotation(direction);

                // if(d< Vector3.Distance(p.First().Source.Node.WorldMatrix.TranslationVector, e.Transform.WorldMatrix.TranslationVector))
                // {
                //     var direction =  e.Transform.Position -  p.First().Source.Node.Transform.Position;
                //     direction.Normalize();
                //     p.ToList().ForEach( v => sk.NodeTransformations[v.Target.Index].Transform.Rotation = DirRotation(direction));
                // }
                // var path = p.ToList();
                // var tmp1 = new FabrikData[path.Count +1];
                // var tmp2 = new FabrikData[path.Count +1];
                // tmp1[0] = new FabrikData{Position = path[0].Source.Node.Transform.Position, Distance = path[0].Source.Distance};
                // for (int i = 1; i < path.Count; i++)
                // {
                //     tmp1[i] = new FabrikData{Position = path[i].Target.Node.Transform.Position, Distance = path[i].Target.Distance};
                // }
                // tmp1[^0] = new FabrikData{Position = e.Transform.Position, Distance = Vector3.Distance(tmp1[^1].Position, e.Transform.Position)};
                // for (int i = 0; i < NbIteration; i++)
                // {
                    
                //     for (int j = tmp1.Length-2; j > 0; j--)
                //     {
                        
                //     }
                //     for (int j = 1; j < tmp1.Length; j++)
                //     {
                //         // tmp1[j] = 
                //     }
                // }
            }
            
        }
        public Quaternion DirRotation(Vector3 dir)
        {
            var m = Matrix.LookAtRH(Vector3.Zero, dir, Vector3.UnitZ);
            return Quaternion.RotationMatrix(m);
        }
        public Matrix LookAt(Vector3 dir)
        {
            return Matrix.LookAtRH(Vector3.Zero, dir, Vector3.UnitY);
            
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
            public Quaternion Rotation;
            public float Distance;
        }
    }
}