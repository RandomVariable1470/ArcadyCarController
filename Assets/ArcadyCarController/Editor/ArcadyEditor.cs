using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine.UIElements;

namespace Arcady.Editor
{
    [CustomEditor(typeof(ArcadyController))]
    public class ArcadyEditor : UnityEditor.Editor
    {
        public VisualTreeAsset VisualTreeAsset;
        
        public override VisualElement CreateInspectorGUI()
        {
            VisualElement root = new VisualElement();

            VisualTreeAsset.CloneTree(root);
            
            return root;
        }
    }
}