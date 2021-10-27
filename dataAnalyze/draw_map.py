#   -- plot --
import matplotlib.pyplot as plt
import matplotlib as mpl
import seaborn as sns
fig = plt.figure(1, (10, 10), dpi=60)
ax = plt.subplot(111)
plt.sca(ax)
fig.tight_layout(rect=(0.05, 0.1, 1, 0.9))  # 调整整体空白
import matplotlib

def sns_draw():
    """
    不能添加权重只用用数量计算多少
    @return:
    """
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('cmap',
                                                               ['#9DCC42', '#FFFE03', '#F7941D', '#E9420E', '#FF0000'], 256)

    plt.axis('off')
    # 定义colorbar位置
    cax = plt.axes([0.13, 0.32, 0.02, 0.3])

    datasample={
        'Lng':[116.62,116.32,116.42,116.43,116.52,116.52],
        'Lat':[31.10,31.20,31.30,31.40,31.60,31.6001],
    }
    # 绘制热力图
    sns.kdeplot(datasample['Lng'], datasample['Lat'],
                alpha=0.8,  # 透明度
                gridsize=1000,  # 绘图精细度，越高越慢
                bw=0.03,  # 高斯核大小（经纬度），越小越精细
                shade=True,
                shade_lowest=False,
                cbar=True,
                cmap=cmap,
                ax=ax,  # 指定绘图位置
                cbar_ax=cax  # 指定colorbar位置
                )

    plt.show()
import scipy
import numpy as np

def heatmapplot(data, weight, gridsize=100, bw='scott', cmap=plt.cm.gist_earth_r, ax=None, **kwargs):
    """

    @param data:
    @param weight:
    @param gridsize:
    @param bw:
    @param cmap:
    @param ax:
    @param kwargs:
    @return:
    """
    # 数据整理
    from scipy import stats
    m1 = data[:, 0]
    m2 = data[:, 1]
    xmin = m1.min()
    xmax = m1.max()
    ymin = m2.min()
    ymax = m2.max()
    X, Y = np.mgrid[xmin:xmax:(xmax - xmin) / gridsize, ymin:ymax:(ymax - ymin) / gridsize]
    positions = np.vstack([X.ravel(), Y.ravel()])
    values = np.vstack([m1, m2])
    # 用scipy计算带权重的高斯kde
    kernel = stats.gaussian_kde(values, bw_method=bw, weights=weight)
    Z = np.reshape(kernel(positions).T, X.shape)
    # 绘制contourf
    cset = ax.contourf(Z.T, extent=[xmin, xmax, ymin, ymax], cmap=cmap, **kwargs)
    # 设置最底层为透明
    cset.collections[0].set_alpha(0)

    return cset
def sci_draw(lng_lat_data,deep_data):
    """
    这个可以添加权重
    @return:
    """
    #   -- plot --
    fig = plt.figure(1, (10, 10), dpi=60)
    ax = plt.subplot(111)
    plt.sca(ax)
    fig.tight_layout(rect=(0.05, 0.1, 1, 0.9))  # 调整整体空白
    # colorbar的数据
    import matplotlib
    # cmap = matplotlib.colors.LinearSegmentedColormap.from_list('cmap',
    #                                                            ['#9DCC42', '#FFFE03', '#F7941D', '#E9420E', '#FF0000'],
    #                                                            256)
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('cmap',
                                                               [(0,'#9DCC42'),  (1,'#FF0000')],
                                                               256)
    bounds = np.linspace(0, 20, 21)
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    # 设定位置
    plt.axis('off')
    # 绘制热力图
    cset = heatmapplot(lng_lat_data,  # 输入经纬度数据
                       deep_data,  # 输入每个点的权重
                       alpha=0.8,  # 透明度
                       gridsize=280,  # 绘图精细度，越高越慢
                       bw=0.03,  # 高斯核大小（经纬度），越小越精细
                       cmap=cmap,
                       ax=ax,
                       # norm=norm
                       )

    # 定义colorbar位置
    cax = plt.axes([0.13, 0.32, 0.02, 0.3])
    plt.colorbar(cset, cax=cax)

    plt.show()

if __name__ == '__main__':
    lng_lat_list = [[116.12, 30.10],
                  [116.22, 30.20],
                  [116.32, 30.30],
                  [116.42, 30.40],
                  [116.52, 31.50],
                  [116.62, 31.6001]]
    lng_lat_data = np.array(lng_lat_list)
    deep_data = [11,3.1,2.7,3,8,5]
    sci_draw(lng_lat_data,deep_data)