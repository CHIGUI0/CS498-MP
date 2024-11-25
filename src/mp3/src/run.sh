# Set up the position 
python3 set_pos.py

# Run the vehicle
python3 vehicle.py &

# Setting
NUM_PARTICLES=500
SENOR_LIMIT=15

# Run the main
python3 main.py \
    --num_particles $NUM_PARTICLES \
    --sensor_limit $SENOR_LIMIT \


    # --measurement_noise true\
