
import opengen as og
import time

period=120

optimizer_path = 'build/drone_controller'
mng = og.tcp.OptimizerTcpManager(optimizer_path)
# Start the server
mng.start()
print("Server started")


print(f"Server running for {period} seconds...")

# Use the solver (returns the inner/outer iter−
# ations, solution time, infeasibility, solution
# status, Lagrange multipliers and solution)

time.sleep(period) # 2 minutes 

print("Killing the server...")
mng.kill()
print("Server killed")