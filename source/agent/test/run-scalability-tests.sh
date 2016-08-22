echo "Building problem generator"
jam -sVARIANTS=OPTIMIZED problem-generator
echo "Generating problems"
./problem-generator_o_rt
echo "Generating input xml"
jam scalability-tests
echo "Running problems"
./problem-generator_o_rt exec
