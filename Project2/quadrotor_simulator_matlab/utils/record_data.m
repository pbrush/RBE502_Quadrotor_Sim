function record_data(state,time,qd_id)

dataset = [time,state];
save("./dataset/quad"+num2str(qd_id),'dataset','qd_id');

end