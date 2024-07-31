// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "colmap/exe/database.h"
#include "colmap/exe/feature.h"
#include "colmap/exe/gui.h"
#include "colmap/exe/image.h"
#include "colmap/exe/model.h"
#include "colmap/exe/mvs.h"
#include "colmap/exe/sfm.h"
#include "colmap/exe/vocab_tree.h"
#include "colmap/util/version.h"

namespace {

typedef std::function<int(int, char**)> command_func_t;

int ShowHelp(
    const std::vector<std::pair<std::string, command_func_t>>& commands) {
  std::cout << colmap::StringPrintf(
                   "%s -- Structure-from-Motion and Multi-View Stereo\n(%s)",
                   colmap::GetVersionInfo().c_str(),
                   colmap::GetBuildInfo().c_str())
            << std::endl
            << std::endl;

  std::cout << "Usage:" << std::endl;
  std::cout << "  colmap [command] [options]" << std::endl << std::endl;

  std::cout << "Documentation:" << std::endl;
  std::cout << "  https://colmap.github.io/" << std::endl << std::endl;

  std::cout << "Example usage:" << std::endl;
  std::cout << "  colmap help [ -h, --help ]" << std::endl;
  std::cout << "  colmap gui" << std::endl;
  std::cout << "  colmap gui -h [ --help ]" << std::endl;
  std::cout << "  colmap automatic_reconstructor -h [ --help ]" << std::endl;
  std::cout << "  colmap automatic_reconstructor --image_path IMAGES "
               "--workspace_path WORKSPACE"
            << std::endl;
  std::cout << "  colmap feature_extractor --image_path IMAGES --database_path "
               "DATABASE"
            << std::endl;
  std::cout << "  colmap exhaustive_matcher --database_path DATABASE"
            << std::endl;
  std::cout << "  colmap mapper --image_path IMAGES --database_path DATABASE "
               "--output_path MODEL"
            << std::endl;
  std::cout << "  ..." << std::endl << std::endl;

  std::cout << "Available commands:" << std::endl;
  std::cout << "  help" << std::endl;
  for (const auto& command : commands) {
    std::cout << "  " << command.first << std::endl;
  }
  std::cout << std::endl;

  return EXIT_SUCCESS;
}

}  // namespace

//主函数入口
int main(int argc, char** argv) {
  colmap::InitializeGlog(argv);
#if defined(COLMAP_GUI_ENABLED)
  Q_INIT_RESOURCE(resources);
#endif

  std::vector<std::pair<std::string, command_func_t>> commands;
  commands.emplace_back("gui", &colmap::RunGraphicalUserInterface);
  commands.emplace_back("automatic_reconstructor",
                        &colmap::RunAutomaticReconstructor);
  commands.emplace_back("bundle_adjuster", &colmap::RunBundleAdjuster);
  commands.emplace_back("color_extractor", &colmap::RunColorExtractor);
  commands.emplace_back("database_cleaner", &colmap::RunDatabaseCleaner);
  commands.emplace_back("database_creator", &colmap::RunDatabaseCreator);
  commands.emplace_back("database_merger", &colmap::RunDatabaseMerger);
  commands.emplace_back("delaunay_mesher", &colmap::RunDelaunayMesher);
  commands.emplace_back("exhaustive_matcher", &colmap::RunExhaustiveMatcher);
  commands.emplace_back("feature_extractor", &colmap::RunFeatureExtractor);
  commands.emplace_back("feature_importer", &colmap::RunFeatureImporter);
  commands.emplace_back("hierarchical_mapper", &colmap::RunHierarchicalMapper);
  commands.emplace_back("image_deleter", &colmap::RunImageDeleter);
  commands.emplace_back("image_filterer", &colmap::RunImageFilterer);
  commands.emplace_back("image_rectifier", &colmap::RunImageRectifier);
  commands.emplace_back("image_registrator", &colmap::RunImageRegistrator);
  commands.emplace_back("image_undistorter", &colmap::RunImageUndistorter);
  commands.emplace_back("image_undistorter_standalone",
                        &colmap::RunImageUndistorterStandalone);
  commands.emplace_back("mapper", &colmap::RunMapper);
  commands.emplace_back("matches_importer", &colmap::RunMatchesImporter);
  commands.emplace_back("model_aligner", &colmap::RunModelAligner);
  commands.emplace_back("model_analyzer", &colmap::RunModelAnalyzer);
  commands.emplace_back("model_comparer", &colmap::RunModelComparer);
  commands.emplace_back("model_converter", &colmap::RunModelConverter);
  commands.emplace_back("model_cropper", &colmap::RunModelCropper);
  commands.emplace_back("model_merger", &colmap::RunModelMerger);
  commands.emplace_back("model_orientation_aligner",
                        &colmap::RunModelOrientationAligner);
  commands.emplace_back("model_splitter", &colmap::RunModelSplitter);
  commands.emplace_back("model_transformer", &colmap::RunModelTransformer);
  commands.emplace_back("patch_match_stereo", &colmap::RunPatchMatchStereo);
  commands.emplace_back("point_filtering", &colmap::RunPointFiltering);
  commands.emplace_back("point_triangulator", &colmap::RunPointTriangulator);
  commands.emplace_back("poisson_mesher", &colmap::RunPoissonMesher);
  commands.emplace_back("project_generator", &colmap::RunProjectGenerator);
  commands.emplace_back("rig_bundle_adjuster", &colmap::RunRigBundleAdjuster);
  commands.emplace_back("sequential_matcher", &colmap::RunSequentialMatcher);
  commands.emplace_back("spatial_matcher", &colmap::RunSpatialMatcher);
  commands.emplace_back("stereo_fusion", &colmap::RunStereoFuser);
  commands.emplace_back("transitive_matcher", &colmap::RunTransitiveMatcher);
  commands.emplace_back("vocab_tree_builder", &colmap::RunVocabTreeBuilder);
  commands.emplace_back("vocab_tree_matcher", &colmap::RunVocabTreeMatcher);
  commands.emplace_back("vocab_tree_retriever", &colmap::RunVocabTreeRetriever);

  // std::vector<std::pair<std::string, command_func_t>> commands;
  // commands.emplace_back("gui", &RunGraphicalUserInterface);   //调用可视化接口
  // commands.emplace_back("automatic_reconstructor", &RunAutomaticReconstructor);  //调用自动重建算法
  // commands.emplace_back("bundle_adjuster", &RunBundleAdjuster);  //只运行全局BA ？ input output ?
  // commands.emplace_back("color_extractor", &RunColorExtractor);  //提取重建后三维点的平均颜色
  // commands.emplace_back("database_creator", &RunDatabaseCreator); //创建一个全局的数据库
  // commands.emplace_back("database_merger", &RunDatabaseMerger);  //运行数据库融合，但是这里融合的是什么东西？
  // commands.emplace_back("delaunay_mesher", &RunDelaunayMesher);  //使用Delaunay三角剖分对稀疏和稠密点云进行网格划分
  // commands.emplace_back("exhaustive_matcher", &RunExhaustiveMatcher);  //计算匹配关系
  // commands.emplace_back("feature_extractor", &RunFeatureExtractor); //提取特征点
  // commands.emplace_back("feature_importer", &RunFeatureImporter);    //导入特征点？ 如何可以导入匹配关系
  // commands.emplace_back("hierarchical_mapper", &RunHierarchicalMapper); //分层次的进行三维重建，先重建子地图，重建子地图之后，进行融合 
  // commands.emplace_back("image_deleter", &RunImageDeleter); //删除某张图片
  // commands.emplace_back("image_filterer", &RunImageFilterer);  //对图片进行滤除
  // commands.emplace_back("image_rectifier", &RunImageRectifier); //立体校正相机，对图片进行去畸变 ？？
  // commands.emplace_back("image_registrator", &RunImageRegistrator); //在地图点中注册一张新的图片
  // commands.emplace_back("image_undistorter", &RunImageUndistorter); //对图片进行去畸变，并且导出一些格式以便于适合PMVS
  // commands.emplace_back("image_undistorter_standalone",
  //                       &RunImageUndistorterStandalone);  //单独的去畸变 ？
  // commands.emplace_back("mapper", &RunMapper);   //稀疏的三维重建
  // commands.emplace_back("matches_importer", &RunMatchesImporter);  //导入匹配关系，这
  // commands.emplace_back("model_aligner", &RunModelAligner);   //运行模型注册？ 给定当前的相机位姿点？
  // commands.emplace_back("model_analyzer", &RunModelAnalyzer);  //打印当前统计的点
  // commands.emplace_back("model_converter", &RunModelConverter);  //将模型转化为对应的格式
  // commands.emplace_back("model_merger", &RunModelMerger);   //融合不同模型
  // commands.emplace_back("model_orientation_aligner",
  //                       &RunModelOrientationAligner);
  // commands.emplace_back("patch_match_stereo", &RunPatchMatchStereo);
  // commands.emplace_back("point_filtering", &RunPointFiltering);  //三维点的过滤
  // commands.emplace_back("point_triangulator", &RunPointTriangulator);
  // commands.emplace_back("poisson_mesher", &RunPoissonMesher);
  // commands.emplace_back("project_generator", &RunProjectGenerator);  //生成工程文件
  // commands.emplace_back("rig_bundle_adjuster", &RunRigBundleAdjuster);  //BA 这个和前面BA的区别？
  // commands.emplace_back("sequential_matcher", &RunSequentialMatcher);
  // commands.emplace_back("spatial_matcher", &RunSpatialMatcher);
  // commands.emplace_back("stereo_fusion", &RunStereoFuser);
  // commands.emplace_back("transitive_matcher", &RunTransitiveMatcher);
  // commands.emplace_back("vocab_tree_builder", &RunVocabTreeBuilder);
  // commands.emplace_back("vocab_tree_matcher", &RunVocabTreeMatcher);
  // commands.emplace_back("vocab_tree_retriever", &RunVocabTreeRetriever);


  if (argc == 1) {
    return ShowHelp(commands);
  }

  const std::string command = argv[1];
  if (command == "help" || command == "-h" || command == "--help") {
    return ShowHelp(commands);
  } else {
    command_func_t matched_command_func = nullptr;
    for (const auto& command_func : commands) {
      if (command == command_func.first) {
        matched_command_func = command_func.second;
        break;
      }
    }
    if (matched_command_func == nullptr) {
      LOG(ERROR) << colmap::StringPrintf(
          "Command `%s` not recognized. To list the "
          "available commands, run `colmap help`.",
          command.c_str());
      return EXIT_FAILURE;
    } else {
      int command_argc = argc - 1;
      char** command_argv = &argv[1];
      command_argv[0] = argv[0];
      return matched_command_func(command_argc, command_argv);
    }
  }

  return ShowHelp(commands);
}
