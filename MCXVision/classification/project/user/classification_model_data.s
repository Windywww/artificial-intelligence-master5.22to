	.section .rodata
	.align 16
	.global num_model_data
	.global num_model_data_end
	.global box_model_data
	.global box_model_data_end

num_model_data:
	.incbin "../user/models/num_cls_npu.tflite"
num_model_data_end:

	.align 16
box_model_data:
	.incbin "../user/models/box_student_npu.tflite"
box_model_data_end:
