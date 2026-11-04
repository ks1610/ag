import os
import pandas as pd
from ultralytics import YOLO

dataset_yaml = '/home/khanhtrinh/trinh/ag/CardboardBox.v1i.yolov8/data.yaml'
output_dir = '/home/khanhtrinh/trinh/ag/model'
epochs_count = 100

os.makedirs(output_dir, exist_ok=True)

model = YOLO('yolov8n.pt')

print("Bắt đầu quá trình huấn luyện...")
results = model.train(
    data=dataset_yaml,
    epochs=epochs_count,
    project=output_dir,       
    name='cardbox_train',     
    exist_ok=True             
)

print("Đang đánh giá mô hình và tạo báo cáo...")
metrics = model.val()

precision = metrics.box.p.mean()
recall = metrics.box.r.mean()
map50 = metrics.box.map50.mean() 

f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

report_data = {
    'Metric': ['Precision', 'Recall', 'F1-Score', 'mAP@50 (Accuracy)'],
    'Value': [f"{precision:.4f}", f"{recall:.4f}", f"{f1_score:.4f}", f"{map50:.4f}"]
}
df_report = pd.DataFrame(report_data)

print("\n--- BẢNG KẾT QUẢ ĐÁNH GIÁ (REPORT) ---")
print(df_report.to_string(index=False))

report_path = os.path.join(output_dir, 'cardbox_train', 'evaluation_report.csv')
df_report.to_csv(report_path, index=False)
print(f"\nFile report đã được lưu thành công tại: {report_path}")