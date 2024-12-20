#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from lerobot.scripts.visualize_dataset_html import visualize_dataset_html


def test_visualize_dataset_html(tmp_path, lerobot_dataset_factory):
    root = tmp_path / "dataset"
    output_dir = tmp_path / "outputs"
    dataset = lerobot_dataset_factory(root=root)
    visualize_dataset_html(
        dataset,
        episodes=[0],
        output_dir=output_dir,
        serve=False,
    )
    assert (output_dir / "static" / "episode_0.csv").exists()
