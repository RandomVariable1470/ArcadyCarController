using UnityEngine;
using UnityEngine.InputSystem;

namespace Arcady
{
    [CreateAssetMenu(fileName = "InputReader", menuName = "Arcady/InputReader")]
    public class InputReader : ScriptableObject
    {
        public Vector2 Move { get; private set; }
        public float Brake { get; private set; }
        
        [SerializeField] private InputActionAsset asset;
        
        private InputAction _moveAction;
        private InputAction _brakeAction;

        public void Enable()
        {
            _moveAction = asset.FindAction("Move");
            _brakeAction = asset.FindAction("Brake");
            
            _moveAction.started += OnMove;
            _brakeAction.started += OnBrake;
            
            _moveAction.performed += OnMove;
            _brakeAction.performed += OnBrake;
            
            _moveAction.canceled += OnMove;
            _brakeAction.canceled += OnBrake;
            
            _moveAction.Enable();
            _brakeAction.Enable();
        }

        public void Disable()
        {
            _moveAction.started -= OnMove;
            _brakeAction.started -= OnBrake;
            
            _moveAction.performed -= OnMove;
            _brakeAction.performed -= OnBrake;
            
            _moveAction.canceled -= OnMove;
            _brakeAction.canceled -= OnBrake;
            
            _moveAction.Disable();
            _brakeAction.Disable();
        }
        
        private void OnMove(InputAction.CallbackContext context)
        {
            Move = context.ReadValue<Vector2>();
        }
        
        private void OnBrake(InputAction.CallbackContext context)
        {
            Brake = context.ReadValue<float>();
        }
    }
}