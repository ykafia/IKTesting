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
using System;

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
            var sk = Entity.Get<ModelComponent>().Skeleton;
            // Quaternion.RotationYawPitchRoll()
            var i = 24;
            var no = sk.NodeTransformations[i];
            var cnPos = sk.NodeTransformations[no.ParentIndex].WorldMatrix.TranslationVector;
            var npos = no.WorldMatrix.TranslationVector;
            var tpos = npos + Vector3.UnitY;
            var p = Matrix.Invert(sk.NodeTransformations[no.ParentIndex].WorldMatrix);
            
            var rot = LookAt(npos, npos - Vector3.UnitX + Vector3.UnitY);
            foreach(var (n,e) in BoneToTarget)
            {
                var rot2 = BDir(cnPos - npos, e.Transform.Position - npos);
                sk.NodeTransformations[i].Transform.Rotation = Quaternion.RotationMatrix(p) * rot2;
            }
            
        }
        public static Quaternion LookAt(Vector3 source, Vector3 target)
        {
            Vector3 forwardVector = Vector3.Normalize(target - source);
            Vector3 rotAxis = Vector3.Cross(-Vector3.UnitZ, forwardVector);
            float dot = Vector3.Dot(-Vector3.UnitZ, forwardVector);
            return Quaternion.Normalize(new Quaternion(rotAxis,dot+1));
        }
        public static Quaternion BDir(Vector3 sourceDir, Vector3 targetDir)
        {
            var t = Quaternion.BetweenDirections(sourceDir,targetDir).YawPitchRoll;
            return Quaternion.RotationYawPitchRoll(t.X,t.Y,0);
        }
        

        
        
        public bool CheckValid()
        {
            var mc = Entity.Get<ModelComponent>();
            return mc != null && mc.Skeleton != null;
        }

        #region InverseKinematicsData

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
        #endregion
    }
}