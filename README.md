# GitHub + Streamlit 数据可视化项目

## 项目概述
基于Streamlit构建的交互式数据可视化Web应用，支持上传/加载数据并生成图表。

## 快速启动
1. 克隆仓库：bash
git clone https://github.com/你的用户名/项目名.git
cd 项目名
2. 安装依赖：bash
pip install -r requirements.txt
3. 运行应用：bash
streamlit run app.py
## 文件说明
- `app.py`：Streamlit主程序
- `data/`：存放数据文件（示例数据需自行准备）
- `requirements.txt`：Python依赖包列表

## 部署到Streamlit Cloud
1. 将项目推送到GitHub仓库。
2. 访问 [streamlit.io/cloud](https://streamlit.io/cloud)，连接GitHub仓库。
3. 选择仓库和主文件（`app.py`）即可自动部署。
