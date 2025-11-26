import numpy as np

# Load the saved NumPy weights + normalization stats
data = np.load("nmpc_nn_controller(tv-1_)(1).npz")

weights = {
    'w0': data['model.0.weight'],
    'b0': data['model.0.bias'],
    'w2': data['model.2.weight'],
    'b2': data['model.2.bias'],
    'w4': data['model.4.weight'],
    'b4': data['model.4.bias']
}
# print("weights", weights['w0'])
mean = data['mean']
std = data['std']
print(f"mean = {mean}")
print(f"std = {std}")
# Define ReLU manually
def relu(x): 
    return np.maximum(0, x)

# Forward function with normalization
def nmpc_forward(x_raw):
    """
    x_raw: shape (1, 3) --> [theta, dtheta, dphi]
    Returns: scalar control input
    """
    # Normalize input
    # mean_override = np.array([0.0, 0.0, 0.0])
    # std_override = np.array([0.5, 5.0, 100.0])
    x = (x_raw - mean) / std
    # Linear + ReLU layers
    x = x @ weights['w0'].T + weights['b0']
    x = relu(x)
    x = x @ weights['w2'].T + weights['b2']
    x = relu(x)
    x = x @ weights['w4'].T + weights['b4']
    
    # Return scalar
    return x

# def nmpc_forwardd(x_raw):
#     if x_raw[0,0] >0.0:
#         return -nmpc_forward(x_raw)
#     else:
#         return nmpc_forward(x_raw)


# Example inference
inp = np.array([[0.05, 0.0]], dtype=np.float32)  # theta, dtheta, dphi
u_pred = nmpc_forward(inp)
print("Predicted control input u:", u_pred)
